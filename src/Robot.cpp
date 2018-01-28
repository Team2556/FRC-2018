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

#ifdef NAVX
#include <AHRS.h>
#endif

#ifdef ADXRS_GYRO
#include <ADXRS450_Gyro.h>
#endif


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
	WPI_TalonSRX *lf = new WPI_TalonSRX(CAN_TALON_LEFT_FRONT); /*left front */
	WPI_TalonSRX *lr = new WPI_TalonSRX(CAN_TALON_LEFT_REAR);/*left rear */
	WPI_TalonSRX *rf = new WPI_TalonSRX(CAN_TALON_RIGHT_FRONT); /*right front */
	WPI_TalonSRX *rr = new WPI_TalonSRX(CAN_TALON_RIGHT_REAR); /*right rear */
	WPI_TalonSRX *am = new WPI_TalonSRX(CAN_TALON_ARM_MOTOR); /*arm motor*/
	WPI_TalonSRX *iom = new WPI_TalonSRX(CAN_TALON_IN_OUT_MOTOR); /*arm motor*/
	//WPI_TalonSRX *cm = new WPI_TalonSRX(CAN_TALON_CLIMB_MOTOR); /*climbing motor*/

	//Setting up solenoid for potential climbing or cube placement on the robot
	//Should be really easy to change for POWER UP robot in the future if something changes
	DoubleSolenoid *	armSolenoid;
	DoubleSolenoid *	climbSolenoid;

	cs::UsbCamera		UsbCamera1;

	DigitalInput *		limitswitch;
	DigitalInput *		limitArm;

	float				fGyroCommandAngle; 	// Gryo angle to seek

	AnalogInput * AnalogIn;

#ifdef NAVX
	AHRS *  			pNavX;
#endif

#ifdef ADXRS_GYRO
	ADXRS450_Gyro *		pADXRS;
#endif

//	AnalogGyro* gyro;


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
		am->Set(ControlMode::PercentOutput, 0);
		iom->Set(ControlMode::PercentOutput, 0);
		//cm->Set(ControlMode::PercentOutput, 0);

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

		// Setup the gyro
#ifdef NAVX
		// Make the NavX control object
		pNavX = new AHRS(SPI::Port::kMXP);
#endif

#ifdef ADXRS_GYRO
		pADXRS = new ADXRS450_Gyro();
#endif

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
#ifdef NAVX
		fGyroCommandAngle = pNavX->GetYaw();
#endif
#ifdef ADXRS_GYRO
		fGyroCommandAngle = pADXRS->GetAngle();
#endif

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

#ifdef NAVX
		fGyroCommandAngle = pNavX->GetYaw();
#endif
#ifdef ADXRS_GYRO
		fGyroCommandAngle = pADXRS->GetAngle();
#endif

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

#if defined(NAVX) or defined(ADXRS_GYRO)
		float			fCurrAngle;
		std::string		sMsg;

#if defined(NAVX)
		fCurrAngle = pNavX->GetYaw();
#elif defined(ADXRS_GYRO)
		fCurrAngle = pADXRS->GetAngle();
#else
		fCurrAngle = fGyroCommandAngle;
#endif
		sMsg = "Yaw = " + std::to_string(fCurrAngle);
		SmartDashboard::PutString("DB/String 6", sMsg.c_str());
		m_robotDrive->DriveCartesian(0.0, 0.0, (fCurrAngle - fGyroCommandAngle) * -0.05, 0.0);
#else
		m_robotDrive->DriveCartesian(0.0, 0.0, 0.0, 0.0);
#endif

#ifdef ADXRS_GYRO
		fGyroCommandAngle = pADXRS->GetAngle();
#endif

		std::string		analog;
		int analogG = AnalogIn->PIDGet();
		analog = std::to_string(analogG);
		SmartDashboard::PutString("DB/String 6", analog.c_str());
	} // end AutonomousPeriodic()


// ----------------------------------------------------------------------------
// Teleop Mode
// ----------------------------------------------------------------------------

	void TeleopInit() {

#ifdef NAVX
		fGyroCommandAngle = pNavX->GetYaw();
#endif
#ifdef ADXRS_GYRO
		fGyroCommandAngle = pADXRS->GetAngle();
#endif
	}


// ----------------------------------------------------------------------------

	void TeleopPeriodic() {
		//driving robot in teleop phase
		/*When programming the drive base you need to call the drivebase that you named in the pointer above.
		//When asigning values it depends on the drive train that you define above and the the subtype, I'm not sure exactly what to call it but in this instace it is DriveCartesian
		//The different values can be set or set with controllers depending on when in the match it is
		//The Different joysticks on the controller can be defined like it is below, something to consider though, the way the WPIlib says to set up a drivetrain like this you ddo GetY the GetX
		//But this is backward. And the foward and backward is inversed so multiplying the joystick value by -1 should fix all errors with this issue*/

		float 			fXStick = 0.0;
		float 			fYStick = 0.0;
		float			fRotate = 0.0;

		// Get drive values from the joystick or the XBox controller
#ifdef JOYSTICK
		fXStick = pclJoystick->GetX();
		fYStick = pclJoystick->GetY() * -1.0;
#endif
#ifdef XBOX
		fXStick = pclXbox->GetX(frc::XboxController::kLeftHand);
		fYStick = pclXbox->GetY(frc::XboxController::kLeftHand) * -1.0;
		fRotate = pclXbox->GetX(frc::XboxController::kRightHand);
#endif

		// If there is a gyro defined then use it
#if defined(NAVX) or defined(ADXRS_GYRO)
		float			fCurrAngle;
		std::string		sMsg;

		// Get current angle from the NavX or ADXRS
#if defined(NAVX)
		fCurrAngle = pNavX->GetYaw();
#elif defined(ADXRS_GYRO)
		fCurrAngle = pADXRS->GetAngle();
#else
		fCurrAngle = fGyroCommandAngle;
#endif

		// Calculate a rotation rate from robot angle error
		sMsg = "Yaw = " + std::to_string(fCurrAngle);
		SmartDashboard::PutString("DB/String 6", sMsg.c_str());
		fRotate = (fCurrAngle - fGyroCommandAngle) * -0.05;
#endif

		// Send drive values to the drive train
		m_robotDrive->DriveCartesian(fXStick, fYStick, fRotate, 0.0);

		//Adding a new pneumatic function for potential climber or gear placement
		//limit switch and manual override
		if(limitswitch->Get()== 0 || pclXbox2->GetXButton()){
					armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
				}

				iom->Set(pclXbox2->GetY(frc::XboxController::kLeftHand));

				armSolenoid->Set(pclXbox2->GetAButton() ? frc::DoubleSolenoid::Value::kReverse : frc::DoubleSolenoid::Value::kOff);

				climbSolenoid->Set(pclXbox2->GetBButton()? frc::DoubleSolenoid::Value::kReverse : frc::DoubleSolenoid::Value::kOff);
				climbSolenoid->Set(pclXbox2->GetYButton() ? frc::DoubleSolenoid::Value::kForward: frc::DoubleSolenoid::Value::kOff);

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
	} // end TeleopPeriodic()


// ----------------------------------------------------------------------------
// Test Mode
// ----------------------------------------------------------------------------

	void TestPeriodic() {}


// ----------------------------------------------------------------------------

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
