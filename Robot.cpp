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
#include "Victor.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "RobotDrive.h"
#include "XboxController.h"
#include "ctre/Phoenix.h"
#include "MotorSafetyHelper.h"
class Robot: public frc::IterativeRobot {

	//const int PDP = 2;
	//const int PCM = 1;


	//Motor controllers set up
	//Victor * 	pclDriveMotorFL;
	//Victor *	pclDriveMotorFR;
	//Victor *	pclDriveMotorBR;
	//Victor * 	pclDriveMotorBL;
	
	//Set up Drive trains
	RobotDrive *	pclRobotDrive;
	RobotDrive *	pclRobotDrive2;

	//set up controllers
	//XboxController* 	pclXbox;
	Joystick *	pclJoystick;
	Joystick *	pclJoystick2;

			/*std::unique_ptr<TalonSRX> _FrontLeft;
			std::unique_ptr<TalonSRX> _FrontRight;
			std::unique_ptr<TalonSRX> _BackLeft;
			std::unique_ptr<TalonSRX> _BackRight;*/

	//set up motor controllers
	WPI_TalonSRX *lf = new WPI_TalonSRX(0); /*left front */
	WPI_TalonSRX *lr = new WPI_TalonSRX(1);/*left rear */
	WPI_TalonSRX *rf = new WPI_TalonSRX(2); /*right front */
	WPI_TalonSRX *rr = new WPI_TalonSRX(3); /*right rear */


			/*const int FRONT_LEFT_MOTOR = 0;
			const int FRONT_RIGHT_MOTOR = 13;

			const int BACK_LEFT_MOTOR = 1;
			const int BACK_RIGHT_MOTOR = 14;*/



public:
	Robot()
		{
			//Initial Drive controllers
			//pclDriveMotorFR = new Victor(1);
			//pclDriveMotorFL = new Victor(0);
			//pclDriveMotorBR = new Victor(2);
			//pclDriveMotorBL = new Victor(3);

		/*_FrontLeft(new TalonSRX(FRONT_LEFT_MOTOR));
		_FrontRight(new TalonSRX(FRONT_RIGHT_MOTOR));
		_BackLeft(new TalonSRX(BACK_LEFT_MOTOR));
		_BackRight(new TalonSRX(BACK_RIGHT_MOTOR));*/

				//reset motor safety timeout//
				lf->Set(ControlMode::PercentOutput, 0);
				lr->Set(ControlMode::PercentOutput, 0);
				rf->Set(ControlMode::PercentOutput, 0);
				rr->Set(ControlMode::PercentOutput, 0);

			//Initial the robot drive
			pclRobotDrive	= new RobotDrive(lf,lr);
			pclRobotDrive2	= new RobotDrive(rf,rr);

			//Initial the joy-stick and inputs
			//pclXbox = new XboxController(0);
			pclJoystick = new Joystick(0);
			pclJoystick2 = new Joystick(1);


		}
	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	}

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

		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {  
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {}

	void TeleopPeriodic() {
		//driving robot in teleop phase
		pclRobotDrive->ArcadeDrive(pclJoystick,0);
		pclRobotDrive2->ArcadeDrive(pclJoystick2,0);
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
