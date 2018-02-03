/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <iostream>
#include <memory>
#include <string>
#include "WPILib.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "RobotDrive.h"
#include "XboxController.h"
#include "ctre/Phoenix.h"
#include "MotorSafetyHelper.h"
#include <DoubleSolenoid.h>
#include "timer.h"
#include "DigitalInput.h"
#include "math.h"

#include "RobotMap.h"
#include "NavGyro.h"


// ============================================================================
// Main iterative robot class
// ============================================================================

class Robot: public frc::IterativeRobot {
	std::unique_ptr<Talon> _LEDs;

	//Set up Drive trains
	MecanumDrive *m_robotDrive;



	//set up controllers
	//Using the two Xbox controllers for competition the Joystick is for potential change in the future todays date is 1/12/18
#ifdef XBOX
	XboxController* 	pclXbox;
	XboxController* 	pclXbox2;
#endif
#ifdef JOYSTICK
	Joystick *			pclJoystick;
#endif

	//set up motor controllers
	//If you need to set up TalonSrx's in the future copy one of these and chanfe the device number found in the ()
	WPI_TalonSRX *lf  = new WPI_TalonSRX(CAN_TALON_LEFT_FRONT);   /* left front     */
	WPI_TalonSRX *lr  = new WPI_TalonSRX(CAN_TALON_LEFT_REAR);    /* left rear      */
	WPI_TalonSRX *rf  = new WPI_TalonSRX(CAN_TALON_RIGHT_FRONT);  /* right front    */
	WPI_TalonSRX *rr  = new WPI_TalonSRX(CAN_TALON_RIGHT_REAR);   /* right rear     */
	WPI_TalonSRX *am  = new WPI_TalonSRX(CAN_TALON_ARM_MOTOR);    /* arm motor      */
	WPI_TalonSRX *iom = new WPI_TalonSRX(CAN_TALON_IN_OUT_MOTOR); /* arm motor      */
	WPI_TalonSRX *wm = new WPI_TalonSRX(CAN_TALON_WRIST_MOTOR); /* arm motor      */
	WPI_TalonSRX *cm = new WPI_TalonSRX(CAN_TALON_CLIMB_MOTOR); /* climbing motor */
	WPI_TalonSRX *cm2 = new WPI_TalonSRX(CAN_TALON_CLIMB_MOTOR2); /* arm motor      */

	//Setting up solenoid for potential climbing or cube placement on the robot
	//Should be really easy to change for POWER UP robot in the future if something changes
	DoubleSolenoid *	armSolenoid;
	DoubleSolenoid *	climbSolenoid;

	cs::UsbCamera		UsbCamera1;

	DigitalInput *		limitswitch;
	DigitalInput *		limitArm;

	float				fGyroCommandAngle; 	// Gryo angle to seek
	bool 				bPresetTurning;

	AnalogInput * 		AnalogIn;
	AnalogInput*		AnalogIn2;

	int					iCommandedArmPosition;	// Position is 0 to 1023

	int positionValue = 0;
	NavGyro	*		pNavGyro;

// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------

// Constructor
// -----------

public:
	Robot()
		{
		//reset motor safety timeout//
		lf->Set(ControlMode::PercentOutput, 0);
		lr->Set(ControlMode::PercentOutput, 0);
		rf->Set(ControlMode::PercentOutput, 0);
		rr->Set(ControlMode::PercentOutput, 0);
		cm->Set(ControlMode::PercentOutput, 0);
		wm->Set(ControlMode::PercentOutput, 0);
		iom->Set(ControlMode::PercentOutput, 0);
		am->Set(ControlMode::PercentOutput, 0);
		cm2->Set(ControlMode::PercentOutput, 0);

    // Setup the Up/Down arm controller
#ifdef ARM_UP_DOWN_USING_POSITION
    // Closed loop tracking with analog voltage as the position sensor
    am->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 0); /* PIDLoop=0,timeoutMs=0 */
    // Dont' ask
    am->ConfigSetParameter(ParamEnum::eFeedbackNotContinuous, 1, 0x00, 0x00, 0x00);
    // Tracking algorithm parmater setup. These need to be correct but can be tricky to get right.
    // These can be adjusted in real time through the roborio web interface.
    am->SelectProfileSlot(0, 0);	// Set the first of two "profile" slots
    am->Config_kF(0, 0.0, 0.0);		// Feed forward term, arm may need a little, we will see
    am->Config_kP(0, 10.0, 0.0);	// Proportional term, play with this first
    am->Config_kI(0, 0.0, 0.0);		// Integration term, play with this next, about 1/1000 of P term is a good start
    am->Config_kD(0, 0.0, 0.0);		// Differentiaion term, probably not needed
//		TalonTest->SetInverted(true);
		am->ConfigPeakCurrentLimit(3.0,0);
//		am->EnableCurrentLimit(true);
    	//am->SetInverted(true);
    am->Set(ControlMode::PercentOutput, 0);	// Set speed control for now, and set speed to zero
#else
    am->Set(ControlMode::PercentOutput, 0);	// Set good ol' speed control
#endif

    // Setup the In/Out arm controller
#ifdef ARM_UP_DOWN_USING_POSITION
// Use the same setup calls as the arm control above
#else
    iom->Set(ControlMode::PercentOutput, 0);
#endif

    //Initial the robot drive
    //Sets the different motor controllers for the drivebase
    m_robotDrive = new MecanumDrive(*lf, *lr, *rf, *rr);

    //sets the safety time out for the motor safety feature on the motors (This is required by FIRST)
    m_robotDrive->SetExpiration(0.5);

    //This is for Power consumption and is not as important but still needs to be in the code
    m_robotDrive->SetSafetyEnabled(false);

    //Initial the joy-stick and inputs as well as the Xbox controllers
    //The values found in the () are for the USB ports that you can change in the Driver Station
#ifdef XBOX
    pclXbox  = new XboxController(0);
    pclXbox2 = new XboxController(1);
#endif

    //The Joystick is for the potential of the future and not for the robot  as of right now 1/12/18
#ifdef JOYSTICK
    pclJoystick = new Joystick(0);
#endif

    //Setting up Pneumatic
    climbSolenoid = new DoubleSolenoid(CAN_PCM, PCM_CHAN_CLIMB_UP, PCM_CHAN_CLIMB_DOWN);
    armSolenoid = new DoubleSolenoid(CAN_PCM, PCM_CHAN_ARM_UP, PCM_CHAN_ARM_UP);

    limitswitch= new DigitalInput(DIO_LIMIT_SW);
    limitswitch= new DigitalInput(DIO_LIMIT_ARM);

		AnalogIn = new AnalogInput(0);
		AnalogIn2 = new AnalogInput(1);

    // Setup the gyro
    pNavGyro = new NavGyro();
    } // end Robot class constructor


// Robot Initialization
// --------------------

void RobotInit() {
    m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
    UsbCamera1 = CameraServer::GetInstance()->StartAutomaticCapture();
    UsbCamera1.SetResolution(160, 120);
    UsbCamera1.SetFPS(5);
    climbSolenoid->Set(frc::DoubleSolenoid::Value::kForward);

    // Get the initial starting angle
    pNavGyro->Init();

} // end RobotInit()


// ----------------------------------------------------------------------------
// Autonomous Mode
// ----------------------------------------------------------------------------

    /*
     * This autonomous (along with the chooser code above) shows how to
     * select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * GetString line to get the auto name from the text box below the Gyro.
     *
     * You can add additional auto modes by adding additional comparisons to
     * the
     * if-else structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as
     * well.
     */
void AutonomousInit() override {
    std::string gameData;

    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

    if(gameData[0] == 'L'){
	SmartDashboard::PutString("DB/String 0", "Left");
    }
    else if(gameData[0]=='R'){
	SmartDashboard::PutString("DB/String 0", "Right");
    }
    else{
	SmartDashboard::PutString("DB/String 0", "UnKnown");
    }

    if(gameData[1] == 'L'){
	SmartDashboard::PutString("DB/String 1", "Left");
    }
    else if(gameData[1]=='R'){
	SmartDashboard::PutString("DB/String 1", "Right");
    }
    else{
	SmartDashboard::PutString("DB/String 1", "UnKnown");
		    }
    if(gameData[2] == 'L'){
	SmartDashboard::PutString("DB/String 2", "Left");
    }
    else if(gameData[2]=='R'){
	SmartDashboard::PutString("DB/String 2", "Right");
    }
    else{
	SmartDashboard::PutString("DB/String 2", "UnKnown");
    }

    pNavGyro->SetCommandYawToCurrent();
    } // end AutonomousInit()



// ----------------------------------------------------------------------------

void AutonomousPeriodic() {
#if 0
    std::string sliderString ;
    double dSliderDrive;
    dSliderDrive = (SmartDashboard::GetNumber("DB/Slider 0", 0.0)-2.5)/2.5;
    sliderString = std::to_string(dSliderDrive);
    SmartDashboard::PutString("DB/String 0", sliderString.c_str());
    m_robotDrive->DriveCartesian(dSliderDrive, 0,0,0 );
    std::string sliderString2 ;
    double dSliderForRev;
    dSliderForRev = (SmartDashboard::GetNumber("DB/Slider 1", 0.0)-2.5)/2.5;
    sliderString = std::to_string(dSliderForRev);
    SmartDashboard::PutString("DB/String 1", sliderString2.c_str());
    m_robotDrive->DriveCartesian(0, dSliderForRev,0,0 );
#endif
#if 0
    float angle = gyro.GetAngle();
    std::string timerShow ;

    double timer = DriverStation::GetInstance().GetMatchTime();
    timerShow = std::to_string(timer);
    SmartDashboard::PutString("DB/String 4", timerShow.c_str());

    double atimer = DriverStation::GetInstance().GetMatchTime();
    if(atimer >= 12.4){
	    m_robotDrive->DriveCartesian(0,0,0,-angle * kP);
    }
    else if((atimer >= 12.2) && (atimer > 0)){
	    m_robotDrive->DriveCartesian(0,1,0,0);
    }
    else if((atimer >= 11.8)&&(atimer > 0)){
	    m_robotDrive->DriveCartesian(1,0,0,0);
    }
    else if((atimer >= 11)&&(atimer > 0)){
	    m_robotDrive->DriveCartesian(0,0,0,0);
    }
#endif

    m_robotDrive->DriveCartesian(0.0, 0.0, pNavGyro->GetYawError() * -0.05, 0.0);

    std::string		analog;
    int analogG = AnalogIn->PIDGet();
    analog = std::to_string(analogG);
    SmartDashboard::PutString("DB/String 6", analog.c_str());
    } // end AutonomousPeriodic()


// ----------------------------------------------------------------------------
// Teleop Mode
// ----------------------------------------------------------------------------

void TeleopInit() {

    pNavGyro->SetCommandYawToCurrent();
//Move to teleop periodic after school
#ifdef ARM_UP_DOWN_USING_POSITION
		// Hold the current arm position where ever it currently is
		iCommandedArmPosition = am->GetSelectedSensorPosition(0);
		am->Set(ControlMode::Position, 120);
#else
    // Turn off the arm motor
    am->Set(ControlMode::PercentOutput, 0);
#endif

    }


// ----------------------------------------------------------------------------

void TeleopPeriodic() {
    float 			fXStick = 0.0;
    float 			fYStick = 0.0;
    float			fRotate = 0.0;
    bool			bAllowRotate = false;

		// Get drive values from the joystick or the XBox controller
#ifdef JOYSTICK
    fXStick = pclJoystick->GetX();
    fYStick = pclJoystick->GetY() * -1.0;
    bAllowRotate = pclJoystick->GetTrigger();
    SmartDashboard::PutString("Rotate", bAllowRotate ? "Yes" : "No ");
#endif
#ifdef XBOX
    fXStick = pclXbox->GetX(frc::XboxController::kLeftHand);
    fYStick = pclXbox->GetY(frc::XboxController::kLeftHand) * -1.0;
    fRotate = pclXbox->GetX(frc::XboxController::kRightHand);
#endif
    if(pclXbox->GetPOV()>-1 && pNavGyro->bPresetTurning == false)
    {
    	pNavGyro->fGyroCommandYaw = pNavGyro->fGyroCommandYaw + pclXbox->GetPOV();
    	pNavGyro->bPresetTurning = true;
    }
    if(fabs(pNavGyro->GetYawError())<10)
    {
    	pNavGyro->bPresetTurning = false;
    }
    // Handle manual rotation
    //bAllowRotate = pclXbox->GetTriggerAxis(frc::XboxController::kRightHand)>.5;
    bAllowRotate = pclXbox->GetX(frc::XboxController::kRightHand)>.1||
    			   pclXbox->GetX(frc::XboxController::kRightHand)<-.1;
    if (bAllowRotate)
	{
	fRotate = pclXbox->GetX(frc::XboxController::kRightHand);
	fRotate = pNavGyro->CorrectRotate(fRotate);
	pNavGyro->SetCommandYawToCurrent();
	}
    else
    {
        // Calculate a rotation rate from robot angle error
    	fRotate = pNavGyro->GetRotate();
    }
    // Send drive values to the drive train
    m_robotDrive->DriveCartesian(fXStick, fYStick, fRotate, 0.0);

		float PsiValue;
		double Vout = AnalogIn2->GetVoltage() ;

		PsiValue = 250*(Vout/2.09384615)-25;
		SmartDashboard::PutNumber("Voltage", AnalogIn2->GetVoltage());
		SmartDashboard::PutNumber("Psi", PsiValue);

		//Adding a new pneumatic function for potential climber or gear placement
		//limit switch and manual override
		if(limitswitch->Get()== 0 || pclXbox2->GetXButton()){
					armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
				}
			iom->Set(pclXbox2->GetY(frc::XboxController::kLeftHand));

    armSolenoid->Set(pclXbox2->GetAButton()   ? frc::DoubleSolenoid::Value::kReverse : frc::DoubleSolenoid::Value::kOff);

    climbSolenoid->Set(pclXbox2->GetBButton() ? frc::DoubleSolenoid::Value::kReverse : frc::DoubleSolenoid::Value::kOff);
    climbSolenoid->Set(pclXbox2->GetYButton() ? frc::DoubleSolenoid::Value::kForward: frc::DoubleSolenoid::Value::kOff);

#ifdef ARM_UP_DOWN_USING_POSITION
				if(pclXbox2->GetBumperPressed(frc::XboxController::kRightHand))
				{
					positionValue++;
				}

				else if(pclXbox2->GetBumperPressed(frc::XboxController::kLeftHand))
				{
					positionValue--;
				}

				else if(positionValue < 0)
				{
					positionValue = 0;
				}

				else if(positionValue > 2)
				{
					positionValue = 2;
				}

				// Put position control code in here. Stay in same position for now. Move arm up and down by
				// changing value of iCommandedArmPosition in code
				if(positionValue == 0)
				{
					am->Set(ControlMode::Position, 100);
					//iom->Set(ControlMode::Position,0);
				}
				else if(positionValue == 1)
				{
					am->Set(ControlMode::Position, 500);
					//iom->Set(ControlMode::Position,500);
				}
				else if(positionValue == 2)
				{
					am->Set(ControlMode::Position, 800);
					//iom->Set(ControlMode::Position,800);
				}
				SmartDashboard::PutNumber("Positoin Value", positionValue);
				SmartDashboard::PutNumber("Pot position",am->GetSelectedSensorPosition(0));

				wm->Set(pclXbox2->GetTriggerAxis(frc::XboxController::kRightHand));
				wm->Set(pclXbox2->GetTriggerAxis(frc::XboxController::kLeftHand)*-1);
#else
    //cm->Set(pclXbox2->GetY(frc::XboxController::kLeftHand));
    if(pclXbox2->GetTriggerAxis(frc::XboxController::kRightHand)> 0.1){
	am->Set(pclXbox2->GetTriggerAxis(frc::XboxController::kRightHand)*-1);
    }
    else if(pclXbox2->GetTriggerAxis(frc::XboxController::kLeftHand)> 0.1){
	am->Set(pclXbox2->GetTriggerAxis(frc::XboxController::kLeftHand));
    }
    else
    {
	am->Set(0);
    }
#endif

} // end TeleopPeriodic()


// ----------------------------------------------------------------------------
// Disabled Mode
// ----------------------------------------------------------------------------

void DisabledInit()
    {
    }


// ----------------------------------------------------------------------------

void DisabledPeriodic()
    {
    }

// ----------------------------------------------------------------------------
// Test Mode
// ----------------------------------------------------------------------------

void TestInit()
    {
//	TalonTest->Set(ControlMode::Position, 512);
    }


// ----------------------------------------------------------------------------

void TestPeriodic()
    {
    }


// ----------------------------------------------------------------------------

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
