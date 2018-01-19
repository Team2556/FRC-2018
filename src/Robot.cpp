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
class Robot: public frc::IterativeRobot {
	std::unique_ptr<Talon> _LEDs;

	const int PDP = 1;
	const int PCM = 4;

	//Set up Drive trains
	MecanumDrive *m_robotDrive;

	//set up controllers
	//Using the two Xbox controllers for competition the Joystick is for potential change in the future todays date is 1/12/18
	XboxController* 	pclXbox;
	XboxController* 	pclXbox2;
	Joystick *	pclJoystick;


	//set up motor controllers
	//If you need to set up TalonSrx's in the future copy one of these and chanfe the device number found in the ()
	WPI_TalonSRX *lf = new WPI_TalonSRX(0); /*left front */
	WPI_TalonSRX *lr = new WPI_TalonSRX(1);/*left rear */
	WPI_TalonSRX *rf = new WPI_TalonSRX(2); /*right front */
	WPI_TalonSRX *rr = new WPI_TalonSRX(3); /*right rear */

	//Setting up solenoid for potential climbing or cube placement on the robot
	//Should be really easy to change for POWER UP robot in the future if something changes
	DoubleSolenoid *	pclSolenoid;

	cs::UsbCamera				UsbCamera1;

	DigitalInput *limitswitch;


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

		//Initial the robot drive
		//Sets the different motor controllers for the drivebase
		m_robotDrive = new MecanumDrive(*lf, *lr, *rf, *rr);

		//sets the safety time out for the motor safety feature on the motors (This is required by FIRST)
		m_robotDrive->SetExpiration(0.5);

		//This is for Power consumption and is not as important but still needs to be in the code
		m_robotDrive->SetSafetyEnabled(false);

		//Initial the joy-stick and inputs as well as the Xbox controllers
		//The values found in the () are for the USB ports that you can change in the Driver Station
		pclXbox  = new XboxController(0);
		pclXbox2 = new XboxController(1);

		//The Joystick is for the potential of the future and not for the robot  as of right now 1/12/18
		pclJoystick = new Joystick(0);

		//Setting up Pneumatic
		pclSolenoid = new DoubleSolenoid(PCM,0,1);

		limitswitch= new DigitalInput(0);

		} // end Robot class constructor


// Robot Initialization
// --------------------

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		UsbCamera1 = CameraServer::GetInstance()->StartAutomaticCapture();
		UsbCamera1.SetResolution(160, 120);
		UsbCamera1.SetFPS(5);
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
					std::string timerShow ;
					double timer = DriverStation::GetInstance().GetMatchTime();
					timerShow = std::to_string(timer);
					SmartDashboard::PutString("DB/String 4", timerShow.c_str());
					double atimer = DriverStation::GetInstance().GetMatchTime();
						if(atimer >= 12.4){
							m_robotDrive->DriveCartesian(-1,1,0,0);
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


	} // end AutonomousPeriodic()


// ----------------------------------------------------------------------------
// Teleop Mode
// ----------------------------------------------------------------------------

	void TeleopInit() {}


// ----------------------------------------------------------------------------

	void TeleopPeriodic() {
		//driving robot in teleop phase
		/*When programming the drive base you need to call the drivebase that you named in the pointer above.
		//When asigning values it depends on the drive train that you define above and the the subtype, I'm not sure exactly what to call it but in this instace it is DriveCartesian
		//The different values can be set or set with controllers depending on when in the match it is
		//The Different joysticks on the controller can be defined like it is below, something to consider though, the way the WPIlib says to set up a drivetrain like this you ddo GetY the GetX
		//But this is backward. And the foward and backward is inversed so multiplying the joystick value by -1 should fix all errors with this issue*/
		m_robotDrive->DriveCartesian(pclXbox->GetX(frc::XboxController::kLeftHand),pclXbox->GetY(frc::XboxController::kLeftHand)*-1,pclXbox->GetX(frc::XboxController::kRightHand),0.0);

		//Adding a new pneumatic function for potential climber or gear placement
		//limit switch and manual override
		if(limitswitch->Get()== 0 || pclXbox2->GetXButton()){
					pclSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
				}

				pclSolenoid->Set(pclXbox2->GetAButton() ? frc::DoubleSolenoid::Value::kReverse : frc::DoubleSolenoid::Value::kOff);

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
