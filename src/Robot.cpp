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
//#include "TargetTrack.h"

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
	//TargetTrack *	pTrack;

	int AutonomousToUse;
	int iTurn = 0;
	int iPath = 0;
	int controlPot = 0;
	float fVisionMaxTargetSize;
	frc::Preferences *pPrefs;
	Relay *pClimbTrigger;

	cs::UsbCamera		UsbCamera1;

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
    //pTrack = new TargetTrack();
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
    //pTrack->Init();
    //SmartDashboard::PutString("Player Station", "M");

    UsbCamera1 = CameraServer::GetInstance()->StartAutomaticCapture();
          UsbCamera1.SetResolution(160, 120);
          UsbCamera1.SetFPS(20);

} // end RobotInit()


// ----------------------------------------------------------------------------
// Autonomous Mode
// ----------------------------------------------------------------------------

#include Auto.inc


// ----------------------------------------------------------------------------
// Teleop Mode
// ----------------------------------------------------------------------------

#include "Teleop.inc"


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
