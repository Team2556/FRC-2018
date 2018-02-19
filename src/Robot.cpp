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
	int					iComandedExtensionPostion;

	int positionValue = 0;
	int stateValue = 0;
	bool limitBool = false;
	int armAuto;
	int armPot;
	NavGyro	*		pNavGyro;

	int AutonomousToUse;
	int iTurn = 0;
	frc::Preferences *pPrefs;

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

    // Closed loop tracking with analog voltage as the position sensor
    am->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 0); /* PIDLoop=0,timeoutMs=0 */
    // Dont' ask
    am->ConfigSetParameter(ParamEnum::eFeedbackNotContinuous, 1, 0x00, 0x00, 0x00);
    // Tracking algorithm parmater setup. These need to be correct but can be tricky to get right.
    // These can be adjusted in real time through the roborio web interface.
    am->SelectProfileSlot(0, 0);	// Set the first of two "profile" slots
   // am->Config_kF(0, 0.0, 0.0);		// Feed forward term, arm may need a little, we will see
  //  am->Config_kP(0, 0.1, 0.0);	// Proportional term, play with this first
   //am->Config_kI(0, 0.0, 0.0);		// Integration term, play with this next, about 1/1000 of P term is a good start
   // am->Config_kD(0, 0.0, 0.0);		// Differentiaion term, probably not needed

//		TalonTest->SetInverted(true);
	//	am->ConfigPeakCurrentLimit(0.01,0);
//		am->EnableCurrentLimit(true);
    	//am->SetInverted(true);
    am->Set(ControlMode::PercentOutput, 0);	// Set speed control for now, and set speed to zero

    am->ConfigOpenloopRamp(0,0);
    lf->ConfigOpenloopRamp(0,0);
    rf->ConfigOpenloopRamp(0,0);
    rr->ConfigOpenloopRamp(0,0);
    lr->ConfigOpenloopRamp(0,0);
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
    armSolenoid = new DoubleSolenoid(CAN_PCM, PCM_CHAN_ARM_UP, PCM_CHAN_ARM_DOWN);

    limitswitch= new DigitalInput(DIO_LIMIT_SW);
    limitArm= new DigitalInput(DIO_LIMIT_ARM);

		AnalogIn = new AnalogInput(0);
		AnalogIn2 = new AnalogInput(1);
	pPrefs = frc::Preferences::GetInstance();
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
    //SmartDashboard::PutString("Player Station", "M");

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
    std::string iPosition;
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

    if(gameData[0] == 'L'){
	SmartDashboard::PutString("Our Switch", "Left");
	AutonomousToUse = 1;
    }
    else if(gameData[0]=='R'){
	SmartDashboard::PutString("Our Switch", "Right");
	AutonomousToUse = 2;
    }
    else{
	SmartDashboard::PutString("Our Switch", "Unknown");
    }

    if(gameData[1] == 'L'){
	SmartDashboard::PutString("Scale", "Left");
    }
    else if(gameData[1]=='R'){
	SmartDashboard::PutString("Scale", "Right");
    }
    else{
	SmartDashboard::PutString("Scale", "UnKnown");
		    }
    if(gameData[2] == 'L'){
	SmartDashboard::PutString("Opposing Switch", "Left");
    }
    else if(gameData[2]=='R'){
	SmartDashboard::PutString("Opposing Switch", "Right");
    }
    else{
	SmartDashboard::PutString("Opposing Switch", "UnKnown");
    }

    iPosition = SmartDashboard::GetString("Player Station", "L");


    if (iPosition == "L" && gameData[0] == 'L')
    {
    	AutonomousToUse = 2;
    }
    else if (iPosition == "L" && gameData[1] == 'L')
	{
		AutonomousToUse = 1;
	}

    if (iPosition == "M" && gameData[0] == 'L')
	{
		AutonomousToUse = 3;
	}
    if (iPosition == "M" && gameData[0] == 'R')
	{
		AutonomousToUse = 4;
	}

    if (iPosition == "R" && gameData[0] == 'R')
	{
		AutonomousToUse = 5;
	}
    else if (iPosition == "R" && gameData[1] == 'R')
	{
		AutonomousToUse = 6;
	}

    lf->ConfigOpenloopRamp(0,0);
    lr->ConfigOpenloopRamp(0,0);
    rf->ConfigOpenloopRamp(0,0);
    rr->ConfigOpenloopRamp(0,0);
    pNavGyro->SetCommandYawToCurrent();
    iTurn = 0;
    //AutonomousToUse = 1;
    } // end AutonomousInit()



// ----------------------------------------------------------------------------

void AutonomousPeriodic()
{
	double xMove = 0;
	double yMove = 0;
	double fRotate = 0;
	double timer = ((DriverStation::GetInstance().GetMatchTime()-7.5)*-1)+7.5;
	SmartDashboard::PutNumber("Timer", timer);
	SmartDashboard::PutNumber("Autonomous Number", AutonomousToUse);
	static int iPath;
	if (AutonomousToUse == 1)
	{
		if (timer<pPrefs->GetFloat("1Path1Start", 0))
		{
			iPath = 0;
		}
		else if (timer>(pPrefs->GetFloat("1Path1Start", 0)) && timer<(pPrefs->GetFloat("1Path1End",0)))
		{
			iPath = 1;
		}
		else if (timer>(pPrefs->GetFloat("1Path2Start", 0)) && timer<(pPrefs->GetFloat("1Path2End",0)))
		{
			iPath =2;
		}
		else if (timer>(pPrefs->GetFloat("1Path3Start", 0)) && timer<(pPrefs->GetFloat("1Path3End",0)))
		{
			iPath = 3;
		}
		else
		{
			iPath = 4;
		}
		if (iPath == 0)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 1)
		{
			xMove = pPrefs->GetDouble("1Path1X",0);
			yMove = pPrefs->GetDouble("1Path1Y",0);
			am->Set(ControlMode::Position, 600);
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
		else if (iPath == 2)
		{
			xMove = pPrefs->GetDouble("1Path2X", 0);
			yMove = pPrefs->GetDouble("1Path2Y", 0);
			am->Set(ControlMode::Position, 600);
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
		else if (iPath == 3)
		{
			xMove = pPrefs->GetDouble("1Path3X", 0);
			yMove = pPrefs->GetDouble("1Path3Y", 0);
			am->Set(ControlMode::Position, 600);
			armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		}


		if (iTurn == 0 && timer>pPrefs->GetDouble("1Turn0Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("1Turn0Amount",0)));
			iTurn++;
		}
		if (iTurn == 1 && timer>pPrefs->GetDouble("1Turn1Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("1Turn1Amount",0)));
			iTurn++;
		}
		if (iTurn == 2 && timer>pPrefs->GetDouble("1Turn2Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("1Turn2Amount",0)));
			iTurn++;
		}
	}

	if (AutonomousToUse == 2)
	{
		if (timer<pPrefs->GetFloat("2Path1Start", 0))
		{
			iPath = 0;
		}
		else if (timer>(pPrefs->GetFloat("2Path1Start", 0)) && timer<(pPrefs->GetFloat("2Path1End",0)))
		{
			iPath = 1;
		}
		else if (timer>(pPrefs->GetFloat("2Path2Start", 0)) && timer<(pPrefs->GetFloat("2Path2End",0)))
		{
			iPath =2;
		}
		else if (timer>(pPrefs->GetFloat("2Path3Start", 0)) && timer<(pPrefs->GetFloat("2Path3End",0)))
		{
			iPath = 3;
		}
		else
		{
			iPath = 4;
		}
		if (iPath == 0)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 1)
		{
			xMove = pPrefs->GetDouble("2Path1X",0);
			yMove = pPrefs->GetDouble("2Path1Y",0);
			am->Set(ControlMode::Position, 500);
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
		else if (iPath == 2)
		{
			xMove = pPrefs->GetDouble("2Path2X", 0);
			yMove = pPrefs->GetDouble("2Path2Y", 0);
			am->Set(ControlMode::Position, 500);
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
		else if (iPath == 3)
		{
			xMove = pPrefs->GetDouble("2Path3X", 0);
			yMove = pPrefs->GetDouble("2Path3Y", 0);
			armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
			am->Set(ControlMode::Position, 500);
		}


		if (iTurn == 0 && timer>pPrefs->GetDouble("2Turn0Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("2Turn0Amount",0)));
			iTurn++;
		}
		if (iTurn == 1 && timer>pPrefs->GetDouble("2Turn1Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("2Turn1Amount",0)));
			iTurn++;
		}
		if (iTurn == 2 && timer>pPrefs->GetDouble("2Turn2Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("2Turn2Amount",0)));
			iTurn++;
		}
	}
	SmartDashboard::PutNumber("Pot position",am->GetSelectedSensorPosition(0));


	if (AutonomousToUse == 3)
	{
		if (timer<pPrefs->GetFloat("3Path1Start", 0))
		{
			iPath = 0;
		}
		else if (timer>(pPrefs->GetFloat("3Path1Start", 0)) && timer<(pPrefs->GetFloat("3Path1End",0)))
		{
			iPath = 1;
		}
		else
		{
			iPath =2;
		}
		if (iPath == 0)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 1)
		{
			xMove = pPrefs->GetDouble("3Path1X",0);
			yMove = pPrefs->GetDouble("3Path1Y",0);
		}
		else if (iPath == 2)
		{
			xMove = pPrefs->GetDouble("3Path2X", 0);
			yMove = pPrefs->GetDouble("3Path2Y", 0);
		}


		if (iTurn == 0 && timer>pPrefs->GetDouble("3Turn0Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("3Turn0Amount",0)));
			iTurn++;
		}
	}
	fRotate = pNavGyro->GetRotate();
	m_robotDrive->DriveCartesian(xMove, yMove, fRotate, 0.0);
	SmartDashboard::PutNumber("Turn Number", iTurn);
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
		iComandedExtensionPostion=iom->GetSelectedSensorPosition(0);
		//am->Set(ControlMode::Position, 120);
#else
    // Turn off the arm motor
    am->Set(ControlMode::PercentOutput, 0);
#endif
    positionValue = pPrefs->GetInt("Position Value", 1);
    }


// ----------------------------------------------------------------------------

void TeleopPeriodic() {
    float 			fXStick = 0.0;
    float 			fYStick = 0.0;
    float			fRotate = 0.0;
    bool			bAllowRotate = false;
    static int 		ArmControlMode;

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
    bAllowRotate = pclXbox->GetTriggerAxis(frc::XboxController::kRightHand)>.5;
    //bAllowRotate = pclXbox->GetX(frc::XboxController::kRightHand)>.1||
    	//		   pclXbox->GetX(frc::XboxController::kRightHand)<-.1;
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
	if(pclXbox2->GetXButton())
	{
		stateValue++;
	}
	else if(limitArm->Get()==0 && pclXbox2->GetAButtonReleased())
	{
		limitBool = true;
	}
	else if(pclXbox2->GetAButtonPressed())
	{
		stateValue--;
		limitBool = false;
	}
	else if(pclXbox2->GetAButtonPressed() && limitArm->Get()==0)
	{
		stateValue--;
		limitBool = false;
	}
	else if(stateValue > 1)
	{
		stateValue = 1;
	}
	else if (stateValue < 0)
	{
		stateValue = 0;
	}


	if(stateValue == 1 || limitBool == true)
	{
		armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
	}
	else if(stateValue == 0 || limitBool == false)
	{
		armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
	}
	else{
		armSolenoid->Set(frc::DoubleSolenoid::Value::kOff);
	}

	SmartDashboard::PutNumber("stateValue", stateValue);
	cm->Set(pclXbox2->GetY(frc::XboxController::kRightHand));
	cm2->Set(pclXbox2->GetY(frc::XboxController::kRightHand));

	SmartDashboard::PutNumber("limitswith 1",limitArm->Get());

    climbSolenoid->Set(pclXbox2->GetYButton() ? frc::DoubleSolenoid::Value::kForward: frc::DoubleSolenoid::Value::kReverse);
    if (pclXbox2->GetStickButtonPressed(frc::XboxController::kRightHand))
    {
    	ArmControlMode--;
    	ArmControlMode = fabs(ArmControlMode);
    	positionValue = -2;
    }
    //pPrefs->PutInt("Preset Position", positionValue);

    wm->Set(pclXbox2->GetY(frc::XboxController::kLeftHand));
    iom->Set(pclXbox2->GetX(frc::XboxController::kLeftHand));

// presets
    if (ArmControlMode == 0)
    {
    	SmartDashboard::PutString("Arm Control Type", "Preset");
		static float fPresetValues[3] = {200, 500, 800};
		if(pclXbox2->GetBumperPressed(frc::XboxController::kRightHand))
		{
			positionValue++;
		}

		else if(pclXbox2->GetBumperPressed(frc::XboxController::kLeftHand))
		{
			positionValue--;
		}

		if(positionValue == -2)
		{
				int currentPositionValue = am->GetSelectedSensorPosition(0);
				int closestPreset = 0;
				int distanceFromClosest = 1000000000;//arbitrary number to make the first value smaller 100% of the time
				for (int i = 0; i<=2; i++)
				{
					int ClosenessTest = fabs(currentPositionValue - fPresetValues[i]);
					if (ClosenessTest<distanceFromClosest)
					{
						distanceFromClosest = ClosenessTest;
						closestPreset = i;
					}
				}
				positionValue = closestPreset;
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
			am->Set(ControlMode::Position, fPresetValues[0]);
			//iom->Set(ControlMode::Position,600);
		}
		else if(positionValue == 1)
		{
			am->Set(ControlMode::Position, fPresetValues[1]);
			//iom->Set(ControlMode::Position,100);
		}
		else if(positionValue == 2)
		{
			am->Set(ControlMode::Position, fPresetValues[2]);
			//iom->Set(ControlMode::Position,800);
		}

		 if(iom->GetSelectedSensorPosition(0)  <=  145)
		       {

		        //	iom->Set(ControlMode::Position,145);
		       }
		     else if(iom->GetSelectedSensorPosition(0) >= 540){
		     //iom->Set(ControlMode::Position,540);
		     }
		 armPot = am->GetSelectedSensorPosition(0);
		SmartDashboard::PutNumber("Positoin Value", positionValue);
		SmartDashboard::PutNumber("Pot position",am->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Pot Position extension" , iom->GetSelectedSensorPosition(0));

    }


	//Manual Control
    else if (ArmControlMode == 1)
    {
    SmartDashboard::PutString("Arm Control Type", "Manual");
    //cm->Set(pclXbox2->GetY(frc::XboxController::kLeftHand));
    if(pclXbox2->GetTriggerAxis(frc::XboxController::kRightHand)> 0.1){
	armPot= armPot+3;
    }
    else if(pclXbox2->GetTriggerAxis(frc::XboxController::kLeftHand)> 0.1){
    	armPot= armPot-3;
    }
    //iom->Set(ControlMode::PercentOutput, pclXbox2->GetX(frc::XboxController::kLeftHand));
    armAuto = 469 - am->GetSelectedSensorPosition(0);
    if(armAuto < 0)
    {
    	armAuto=armAuto*-1;
    }

    if(armAuto > 469)
    {
    	armAuto = armAuto * 2.05;
    }
    else if(armAuto < 469)
    {
    	armAuto = armAuto * 3.5;
    }
    else{
    	armAuto = 0;
    }
    am->Set(ControlMode::Position,armPot);
    //iom->Set(ControlMode::Position,armAuto);


    }

    SmartDashboard::PutNumber("Positoin Value", positionValue);
    SmartDashboard::PutNumber("Pot position",am->GetSelectedSensorPosition(0));
    SmartDashboard::PutNumber("Pot Position extension" , iom->GetSelectedSensorPosition(0));
    SmartDashboard::PutNumber("armAuto: ",armAuto);
    SmartDashboard::PutNumber("armPot: ",armPot);
    SmartDashboard::PutNumber("Front Left", lf->GetMotorOutputPercent());
    SmartDashboard::PutNumber("Front Right", rf->GetMotorOutputPercent());
    SmartDashboard::PutNumber("Back Left", lr->GetMotorOutputPercent());
    SmartDashboard::PutNumber("Back Right", rr->GetMotorOutputPercent());


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
