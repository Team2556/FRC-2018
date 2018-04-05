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
#include <Preferences.h>

#include "RobotMap.h"
#include "NavGyro.h"
#include "TargetTrack.h"

#define VISION

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
	TargetTrack *	pTrack;

	int AutonomousToUse;
	int iTurn = 0;
	int iPath = 0;
	int controlPot = 0;
	float fVisionMaxTargetSize;
	frc::Preferences *pPrefs;
	Relay *pClimbTrigger;

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
    pTrack = new TargetTrack();
    pClimbTrigger = new Relay(RELAY_CHAN_TRIGGER,frc::Relay::Direction::kReverseOnly);
    } // end Robot class constructor


// Robot Initialization
// --------------------

void RobotInit() {
    m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
    climbSolenoid->Set(frc::DoubleSolenoid::Value::kForward);

    // Get the initial starting angle
    pNavGyro->Init();
    pTrack->Init();
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
    int iPosition;
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

    if(gameData[0] == 'L'){
	SmartDashboard::PutString("Our Switch", "Left");
    }
    else if(gameData[0]=='R'){
	SmartDashboard::PutString("Our Switch", "Right");
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

    iPosition = pPrefs->GetInt("Player Station", 0);// 1 is left, 2 is mid, 3 is right
/*if ((pPrefs->GetBoolean("Vision Auto", true))==false)
{
	if(gameData[])
}
else */

    // Default is to just drive forward past the line
    // AutonomousToUse = XXX

if (pPrefs->GetInt("AutoPriority", 0) == 0)//0 is scale, 1 is switch
{
    if (iPosition == 1 && gameData[1] == 'L')
    {
    	AutonomousToUse = 1;
    }
    else if (iPosition == 1 && gameData[0] == 'L')
	{
		AutonomousToUse = 2;
	}
    else if (iPosition == 2 && gameData[0] == 'L')
	{
		AutonomousToUse = 3;
	}
    else if (iPosition == 2 && gameData[0] == 'R')
	{
		AutonomousToUse = 4;
	}

    else if (iPosition == 3 && gameData[1] == 'R')
	{
		AutonomousToUse = 6;
	}
    else if (iPosition == 3 && gameData[0] == 'R')
	{
		AutonomousToUse = 5;
	}
    else
    {
    	AutonomousToUse = 7;
	}
}
else
{
	if (iPosition == 1 && gameData[0] == 'L')
	{
		AutonomousToUse = 2;
	}
	else if (iPosition == 1 && gameData[1] == 'L')
	{
		AutonomousToUse = 1;
	}
	else if (iPosition == 2 && gameData[0] == 'L')
	{
		AutonomousToUse = 3;
	}
	else if (iPosition == 2 && gameData[0] == 'R')
	{
		AutonomousToUse = 4;
	}

	else if (iPosition == 3 && gameData[0] == 'R')
	{
		AutonomousToUse = 5;
	}
	else if (iPosition == 3 && gameData[1] == 'R')
	{
		AutonomousToUse = 6;
	}
	else
	{
		AutonomousToUse = 7;
	}
}

    lf->ConfigOpenloopRamp(0,0);
    lr->ConfigOpenloopRamp(0,0);
    rf->ConfigOpenloopRamp(0,0);
    rr->ConfigOpenloopRamp(0,0);
    pNavGyro->SetCommandYawToCurrent();
    iTurn = 0;
    iPath = 0;
    fVisionMaxTargetSize = pPrefs->GetFloat("Vision Max Target Size", 0.35);
    pPrefs->PutFloat("Vision Max Target Size", fVisionMaxTargetSize);
    printf("Finished Auto Init");
    } // end AutonomousInit()



// ----------------------------------------------------------------------------

void AutonomousPeriodic()
{
	static bool bStartedTracking = false;
	double xMove = 0;
	double yMove = 0;
	double fRotate = 0;
#define ARM_POT_SCALE 535
#define IO_POT_SCALE 480
#define ARM_POT_SWITCH 335
#define IO_POT_SWITCH 100
#define ARM_POT_END 200
#define IO_POT_END 100
    float  fTrackErrorX, fTrackErrorY;
    float  fTargetSizeX, fTargetSizeY;
	double timer = (((DriverStation::GetInstance().GetMatchTime()-7.5)*-1)+7.5)-2;

	SmartDashboard::PutNumber("Path", iPath);
	//SmartDashboard::PutNumber("NavX Angle", fTargetSizeX);
	SmartDashboard::PutNumber("Timer", timer);
		SmartDashboard::PutNumber("Autonomous Number", AutonomousToUse);

	printf("Got Here");
	bool bTrackLock;
	bTrackLock = pTrack->GetTrackError(&fTrackErrorX, &fTrackErrorY, &fTargetSizeX, &fTargetSizeY);
	SmartDashboard::PutNumber("Target Size", fTargetSizeX);
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
		else if (timer>(pPrefs->GetFloat("1Path4Start", 0)) && timer<(pPrefs->GetFloat("1Path4End",0)))
		{
			iPath = 4;
		}
		else
		{
			iPath = 5;
		}


		if (iPath == 0)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 1)
		{
			xMove = 0;
			yMove = .5;
		}
		else if (iPath == 2)
		{
			xMove = 0;
			yMove = .25;
		}
		else if (iPath == 3)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 4)
		{
			xMove = 0;
			yMove = 0;
		}
		else
		{
			xMove = 0;
			yMove = 0;
		}


		if (iTurn == 0 && timer>pPrefs->GetDouble("1Turn0Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("1Turn0Amount",0)));
			iTurn++;
		}
		if (iTurn == 1 && timer>pPrefs->GetDouble("1Turn1Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(90));
			iTurn++;
		}
		if (iTurn == 2 && timer>pPrefs->GetDouble("1Turn2Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("1Turn2Amount",0)));
			iTurn++;
		}
		if (iTurn == 3 && timer>pPrefs->GetDouble("1Turn3Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("1Turn3Amount",0)));
			iTurn++;
		}


		if (timer<pPrefs->GetDouble("1CubeDropPoint", 0))
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		}
		else
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}

			am->Set(ControlMode::Position, ARM_POT_SCALE);
			if(am->GetSelectedSensorPosition(0)>=ARM_POT_SCALE)
			{
			iom->Set(.75);
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
		else if (timer>(pPrefs->GetFloat("2Path4Start", 0)) && timer<(pPrefs->GetFloat("2Path4End",0)))
		{
			iPath = 4;
		}
		else
		{
			iPath = 5;
		}


		if (iPath == 0)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 1)
		{
			xMove = 0;
			yMove = .75;
		}
		else if (iPath == 2)
		{
			yMove = .25;
			xMove = 0;
		}
		else if (iPath == 3)
		{
			xMove = -.75;
			yMove = 0;
		}
		else if (iPath == 4)
		{
			xMove = 0;
			yMove = 0;
		}
		else
		{
			xMove = 0;
			yMove = 0;
		}


		if (iTurn == 0 && timer>pPrefs->GetDouble("2Turn0Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("2Turn0Amount",0)));
			iTurn++;
		}
		if (iTurn == 1 && timer>pPrefs->GetDouble("2Turn1Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+90);
			iTurn++;
		}


		if (timer<pPrefs->GetDouble("2CubeDropPoint", 0))
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		}
		else
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
			am->Set(ControlMode::Position, ARM_POT_SWITCH);
			//iom->Set(ControlMode::Position, IO_POT_SWITCH);
	}

	// Left vision target from center
	if (AutonomousToUse == 3)
	{
		// No movement
		if (iPath<=0 && timer<pPrefs->GetFloat("3Path1Start", 0))
		{
			iPath = 0;
		}
		// Drive forward
		else if (timer>(pPrefs->GetFloat("3Path1Start", 0)) && timer<(pPrefs->GetFloat("3Path1End",.9)))
		{
			iPath = 1;
		}
		// Strafe left until target acquired
		else if(iPath<=2 && timer>(pPrefs->GetFloat("3Path2Start", .9)) && (bTrackLock) == false)
		{
			iPath = 2;
		}

		else if (iPath<=3)
		{
			iPath = 3;
		}


		if (iPath == 0)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 1)
		{
			xMove = -.5;
			yMove = .3;
			//armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
			am->Set(ControlMode::Position, 500);
		}
		else if (iPath == 2)
		{
			xMove = -.7;
			yMove = 0;
			//armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
			am->Set(ControlMode::Position, 500);
		}
		else if (iPath == 3)
		{
			if (bTrackLock && fTrackErrorX < 0.0)
			{
				iPath = 4;
			}
			else
			{
				xMove = -.2;
				yMove = 0;
				//armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
				printf("Can't Track\n");
			}
			am->Set(ControlMode::Position, 500);
		}
		else if (iPath == 4)
		{
			if (fTargetSizeX < fVisionMaxTargetSize)
			{
				xMove = fTrackErrorX * 1.0;
				yMove = (fVisionMaxTargetSize - fTargetSizeX) * 4;
				yMove = fmin(yMove,  0.25);
				yMove = fmax(yMove, -0.25);
				//armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
			}
			else
			{
			//	armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
				iPath = 5;
			}
		}
		else
		{
			xMove = 0;
			yMove = 0;
		}
		am->Set(ControlMode::Position, ARM_POT_SWITCH);
		//iom->Set(ControlMode::Position, IO_POT_SWITCH);

		if(bTrackLock && iPath>=3)
		{
			bStartedTracking = true;
		}
		if(bStartedTracking && timer>10 && bTrackLock)
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
		else if (timer>5)
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
		else
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		}
	}

	// Right vision target from center
	if (AutonomousToUse == 4)
	{
		// No movement
		if (iPath<=0 && timer<pPrefs->GetFloat("4Path1Start", 0))
		{
			iPath = 0;
		}
		// Drive forward
		else if (timer>(pPrefs->GetFloat("4Path1Start", 0)) && timer<(pPrefs->GetFloat("4Path1End",.9)))
		{
			iPath = 1;
		}
		// Strafe until target aquired
		else if(iPath<=2 && timer>(pPrefs->GetFloat("4Path2Start", .9)) && (bTrackLock) == false)
		{
			iPath =2;
		}
		// Drive towards target
		else if (iPath<=3)
		{
			iPath = 3;
		}


		if (iPath == 0)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 1)
		{
			xMove = .5; // Maybe xMove = 0.5;
			yMove = .3;
			//armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
			am->Set(ControlMode::Position, 500);
		}
		else if (iPath == 2)
		{
			xMove = .7;
			yMove = 0;
			//armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
			am->Set(ControlMode::Position, 500);
		}
		else if (iPath == 3)
		{
			if (bTrackLock && fTrackErrorX > 0.0)
			{
				iPath = 4;
			}
			else
			{
				xMove = .2;
				yMove = 0;
				//armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
				printf("Can't Track\n");
			}
			am->Set(ControlMode::Position, 500);
		}
		else if (iPath == 4)
		{
			if (fTargetSizeX < fVisionMaxTargetSize)
			{
				xMove = fTrackErrorX * 1.0;
				yMove = (fVisionMaxTargetSize - fTargetSizeX) * 6;
				yMove = fmin(yMove,  0.25);
				yMove = fmax(yMove, -0.25);
				//armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
			}
			else
			{
				//armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
				iPath = 5;
			}
		}
		else
		{
			xMove = 0;
			yMove = 0;
		}
		am->Set(ControlMode::Position, ARM_POT_SWITCH);
		//iom->Set(ControlMode::Position, IO_POT_SWITCH);

		if(bTrackLock && iPath>=3)
		{
			bStartedTracking = true;
		}
		if(bStartedTracking && timer>10 && bTrackLock)
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
		else if (timer>5)
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
		else
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		}
	}
	if (AutonomousToUse == 5)
	{
		if (timer<pPrefs->GetFloat("5Path1Start", 0))
		{
			iPath = 0;
		}
		else if (timer>(pPrefs->GetFloat("5Path1Start", 0)) && timer<(pPrefs->GetFloat("5Path1End",0)))
		{
			iPath = 1;
		}
		else if (timer>(pPrefs->GetFloat("5Path2Start", 0)) && timer<(pPrefs->GetFloat("5Path2End",0)))
		{
			iPath =2;
		}
		else if (timer>(pPrefs->GetFloat("5Path3Start", 0)) && timer<(pPrefs->GetFloat("5Path3End",0)))
		{
			iPath = 3;
		}
		else if (timer>(pPrefs->GetFloat("5Path4Start", 0)) && timer<(pPrefs->GetFloat("5Path4End",0)))
		{
			iPath = 4;
		}
		else
		{
			iPath = 5;
		}


		if (iPath == 0)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 1)
		{
			xMove = 0;
			yMove = .75;
		}
		else if (iPath == 2)
		{
			xMove = 0;
			yMove = .25;
		}
		else if (iPath == 3)
		{
			xMove = .75;
			yMove = 0;
		}
		else if (iPath == 4)
		{
			xMove = 0;
			yMove = 0;
		}
		else
		{
			xMove = 0;
			yMove = 0;
		}


		if (iTurn == 0 && timer>pPrefs->GetDouble("5Turn0Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("5Turn0Amount",0)));
			iTurn++;
		}
		if (iTurn == 1 && timer>pPrefs->GetDouble("5Turn1Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw-90);
			iTurn++;
		}


		if (timer<pPrefs->GetDouble("5CubeDropPoint", 0))
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		}
		else
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
			am->Set(ControlMode::Position, ARM_POT_SWITCH);
			//iom->Set(ControlMode::Position, IO_POT_SWITCH);

	}


	if (AutonomousToUse == 6)
	{
		if (timer<pPrefs->GetFloat("6Path1Start", 0))
		{
			iPath = 0;
		}
		else if (timer>(pPrefs->GetFloat("6Path1Start", 0)) && timer<(pPrefs->GetFloat("6Path1End",0)))
		{
			iPath = 1;
		}
		else if (timer>(pPrefs->GetFloat("6Path2Start", 0)) && timer<(pPrefs->GetFloat("6Path2End",0)))
		{
			iPath =2;
		}
		else if (timer>(pPrefs->GetFloat("6Path3Start", 0)) && timer<(pPrefs->GetFloat("6Path3End",0)))
		{
			iPath = 3;
		}
		else if (timer>(pPrefs->GetFloat("6Path4Start", 0)) && timer<(pPrefs->GetFloat("6Path4End",0)))
		{
			iPath = 4;
		}
		else
		{
			iPath = 5;
		}


		if (iPath == 0)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 1)
		{
			xMove = 0;
			yMove = .5;
		}
		else if (iPath == 2)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 3)
		{
			xMove = 0;
			yMove = 0;
		}
		else if (iPath == 4)
		{
			xMove = 0;
			yMove = 0;
		}
		else
		{
			xMove = 0;
			yMove = 0;
		}


		if (iTurn == 0 && timer>pPrefs->GetDouble("6Turn0Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("6Turn0Amount",0)));
			iTurn++;
		}
		if (iTurn == 1 && timer>pPrefs->GetDouble("6Turn1Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw-90);
			iTurn++;
		}
		if (iTurn == 2 && timer>pPrefs->GetDouble("6Turn2Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("6Turn2Amount",0)));
			iTurn++;
		}
		if (iTurn == 3 && timer>pPrefs->GetDouble("6Turn3Start", 0))
		{
			pNavGyro->SetCommandYaw(pNavGyro->fGyroCommandYaw+(pPrefs->GetDouble("6Turn3Amount",0)));
			iTurn++;
		}


		if (timer<pPrefs->GetDouble("6CubeDropPoint", 0))
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		}
		else
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
		am->Set(ControlMode::Position, ARM_POT_SCALE);
					if(am->GetSelectedSensorPosition(0)>ARM_POT_SCALE)
					{
					iom->Set(.75);
					}
		/*if (timer<(pPrefs->GetDouble("6CubeDropPoint", 0)+3))
		{
			am->Set(ControlMode::Position, ARM_POT_SCALE);
			iom->Set(ControlMode::Position, IO_POT_SCALE);
		}
		else
		{
			static int i = am->GetSelectedSensorPosition(0);
			if (fabs(i-ARM_POT_END)<10)
			{
				iom->Set(ControlMode::Position, 500);
			}
			else if(i>ARM_POT_END)
			{
				i = i- 12;

			am->Set(ControlMode::Position, i);
			iom->Set(ControlMode::Position, IO_POT_END);
			}
		}*/
	}
	if (AutonomousToUse == 7)
	{
		if (timer>0&&timer<4)
		{
			xMove = 0;
			yMove = .5;
		}
		else
		{
			xMove = 0;
			yMove = 0;
		}
	}
	fRotate = pNavGyro->GetRotate();

	m_robotDrive->DriveCartesian(xMove, yMove, fRotate, 0.0);
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
    controlPot = 200;
	armPot = am->GetSelectedSensorPosition(0);
	stateValue = 1;
    }


// ----------------------------------------------------------------------------

void TeleopPeriodic() {
	pNavGyro->UpdateValues();
	float  fTrackErrorX, fTrackErrorY;
    float  fTargetSizeX, fTargetSizeY;
    bool bTrackLock;
    bTrackLock = pTrack->GetTrackError(&fTrackErrorX, &fTrackErrorY, &fTargetSizeX, &fTargetSizeY);
    SmartDashboard::PutNumber("Target Size", fTargetSizeX);
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
    fXStick = pclXbox->GetX(frc::XboxController::kLeftHand)*0.7;
    fYStick = (pclXbox->GetY(frc::XboxController::kLeftHand) * -1.0)*0.7;
    fRotate = pclXbox->GetX(frc::XboxController::kRightHand)*0.7;
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
    if (!bAllowRotate)
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

    static bool bDirectionNormal = true;
    SmartDashboard::PutBoolean("Direction Normal", bDirectionNormal);
    if (pclXbox->GetYButton())
    {
    	bDirectionNormal = true;
    }
    else if (pclXbox->GetBButton())
    {
    	bDirectionNormal = false;
    }
    // Send drive values to the drive train
    if(bDirectionNormal)
    {
    	m_robotDrive->DriveCartesian(fXStick, fYStick, fRotate, 0.0);
    }
    else
    {
    	m_robotDrive->DriveCartesian(fYStick, -fXStick, fRotate,0);
    }
		float PsiValue;
		double Vout = AnalogIn2->GetVoltage() ;

		PsiValue = 250*(Vout/2.09384615)-25;
		//SmartDashboard::PutNumber("Psi", PsiValue);

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
		armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
	}
	else if(stateValue == 0 || limitBool == false)
	{
		armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
	}
	else{
		armSolenoid->Set(frc::DoubleSolenoid::Value::kOff);
	}

//	SmartDashboard::PutNumber("stateValue", stateValue);
	cm->Set(pclXbox2->GetY(frc::XboxController::kRightHand));
	cm2->Set(pclXbox2->GetY(frc::XboxController::kRightHand));

	SmartDashboard::PutNumber("limitswitch 1",limitArm->Get());

	climbSolenoid->Set(pclXbox2->GetYButton() ? frc::DoubleSolenoid::Value::kReverse : frc::DoubleSolenoid::Value::kOff);
	//pClimbTrigger->Set(pclXbox2->GetStartButton() ? frc::Relay::Value::kReverse : frc::Relay::Value::kOff);
    //pPrefs->PutInt("Preset Position", positionValue);
    if(pclXbox2->GetY(frc::XboxController::kLeftHand)<-0.4)
    {
    wm->Set(pclXbox2->GetY(frc::XboxController::kLeftHand));
    }
    else if(pclXbox2->GetY(frc::XboxController::kLeftHand)>0.4)
    {
    	 wm->Set(pclXbox2->GetY(frc::XboxController::kLeftHand));
    }
    else
    {
    	wm->Set(0);
    }


	//Manual Control

    SmartDashboard::PutString("Arm Control Type", "Manual");
    //cm->Set(pclXbox2->GetY(frc::XboxController::kLeftHand));
    if(pclXbox2->GetTriggerAxis(frc::XboxController::kRightHand)> 0.1){
	armPot= armPot+15;
    }
    else if(pclXbox2->GetTriggerAxis(frc::XboxController::kLeftHand)> 0.1){
    	armPot= armPot-15;
    }

    /*if(pclXbox2->GetYButtonPressed())
        {
    	am->Set(ControlMode::Position, armPot);
        }*/
    //iom->Set(ControlMode::PercentOutput, pclXbox2->GetX(frc::XboxController::kLeftHand));
   /*if(am->GetSelectedSensorPosition(0) > 750 || am->GetSelectedSensorPosition(0) < 270)
   {
	   iom->Set(pclXbox2->GetX(frc::XboxController::kLeftHand));
   }
   else
   {
	   iom->Set(ControlMode::Position,95);
   }*/
    am->Set(ControlMode::Position,armPot);
    //iom->Set(ControlMode::Position,armAuto);

    if(am->GetSelectedSensorPosition(0) >=pPrefs->GetInt("Upper Grab Height", 1023) && am->GetSelectedSensorPosition(0) <=pPrefs->GetInt("Upper Scale Height", 1023) )
    {
    	iom->Set(ControlMode::Position, 585);
    }
    else if( (am->GetSelectedSensorPosition(0) < pPrefs->GetInt("Upper Grab Height", 1023) &&  am->GetSelectedSensorPosition(0) > pPrefs->GetInt("Lower Grab Height", 0))|| am->GetSelectedSensorPosition(0) > pPrefs->GetInt("Upper Scale Height", 1023))
    {
    	 if (pclXbox2->GetBumper(frc::XboxController::kLeftHand) && pclXbox2->GetBumper(frc::XboxController::kRightHand) )
    	        {
    	        	iom->Set(ControlMode::Position, 360);
    	        }
    	    else if(pclXbox2->GetBumper(frc::XboxController::kRightHand))
    	    {
    	    	iom->Set(ControlMode::Position, 585);
    	    }
    	    else if(pclXbox2->GetBumper(frc::XboxController::kLeftHand))
    	    {
    	    	iom->Set(ControlMode::Position, 54);
    	    }
    	    else
    	    {
    	    	iom->Set(ControlMode::PercentOutput, 0);
    	    }
    }
    else
    {
    	iom->Set(0);
    }




    //iom->Set(ControlMode::Position, 369);
   // SmartDashboard::PutNumber("Positoin Value", positionValue);
    SmartDashboard::PutNumber("Pot position",am->GetSelectedSensorPosition(0));
    SmartDashboard::PutNumber("Pot Position extension" , iom->GetSelectedSensorPosition(0));
   // SmartDashboard::PutNumber("armPot: ",armPot);
    SmartDashboard::PutNumber("Nav-X", pNavGyro->GetDisplacemetX());
    SmartDashboard::PutNumber("Nav-Y", pNavGyro->GetDisplacemetY());
    SmartDashboard::PutNumber("Nav-Z", pNavGyro->GetDisplacemetZ());



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
