
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

if (pPrefs->GetInt("AutoPriority", 0) == 0)//0 is scale, 1 is switch, 2 is will not do scale but will do switch
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
else if (pPrefs->GetInt("AutoPriority", 0) == 1)
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
else if (pPrefs->GetInt("AutoPriority", 0) == 2)
{
	if(iPosition == 1 && gameData[0] =='L')
	{
		AutonomousToUse = 2;
	}
	else if(iPosition == 3 && gameData[0] == 'R')
	{
		AutonomousToUse = 5;
	}
	else if (iPosition == 2 && gameData[0] == 'L')
	{
		AutonomousToUse = 3;
	}
	else if (iPosition == 2 && gameData[0] == 'R')
	{
		AutonomousToUse = 4;
	}
	else
	{
		AutonomousToUse =7;
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
//	static bool bStartedTracking = false;
	double xMove = 0;
	double yMove = 0;
	double fRotate = 0;
#define ARM_POT_SCALE 670
#define IO_POT_SCALE 480
#define ARM_POT_SWITCH 430
#define IO_POT_SWITCH 100
#define ARM_POT_END 200
#define IO_POT_END 100
//    float  fTrackErrorX, fTrackErrorY;
//    float  fTargetSizeX, fTargetSizeY;
	double timer = (((DriverStation::GetInstance().GetMatchTime()-7.5)*-1)+7.5)-2;

	SmartDashboard::PutNumber("Path", iPath);
	//SmartDashboard::PutNumber("NavX Angle", fTargetSizeX);
	SmartDashboard::PutNumber("Timer", timer);
		SmartDashboard::PutNumber("Autonomous Number", AutonomousToUse);

	printf("Got Here");
//	bool bTrackLock;
	//bTrackLock = pTrack->GetTrackError(&fTrackErrorX, &fTrackErrorY, &fTargetSizeX, &fTargetSizeY);
//	SmartDashboard::PutNumber("Target Size", fTargetSizeX);
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
			yMove = .1;
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
			if(am->GetSelectedSensorPosition(0)>=ARM_POT_SCALE-20)
			{
			iom->Set(ControlMode::Position, pPrefs->GetInt("IOM Out", 50));
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
			yMove = .1;
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
/*
	// Middle Vision Auto
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
	}*/
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
		else if (timer>(pPrefs->GetFloat("3Path2Start", 0)) && timer<(pPrefs->GetFloat("3Path2End",0)))
		{
			iPath =2;
		}
		else if (timer>(pPrefs->GetFloat("3Path3Start", 0)) && timer<(pPrefs->GetFloat("3Path3End",0)))
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
			xMove = -.5;
			yMove = .3;
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
		else
		{
			xMove = 0;
			yMove = 0;
		}

		if (timer<pPrefs->GetDouble("3CubeDropPoint", 0))
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		}
		else
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
			am->Set(ControlMode::Position, ARM_POT_SWITCH);
	}
	if (AutonomousToUse == 4)
	{
		if (timer<pPrefs->GetFloat("4Path1Start", 0))
		{
			iPath = 0;
		}
		else if (timer>(pPrefs->GetFloat("4Path1Start", 0)) && timer<(pPrefs->GetFloat("4Path1End",0)))
		{
			iPath = 1;
		}
		else if (timer>(pPrefs->GetFloat("4Path2Start", 0)) && timer<(pPrefs->GetFloat("4Path2End",0)))
		{
			iPath =2;
		}
		else if (timer>(pPrefs->GetFloat("4Path3Start", 0)) && timer<(pPrefs->GetFloat("4Path3End",0)))
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
			xMove = .5;
			yMove = .3;
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
		else
		{
			xMove = 0;
			yMove = 0;
		}

		if (timer<pPrefs->GetDouble("4CubeDropPoint", 0))
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		}
		else
		{
			armSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		}
			am->Set(ControlMode::Position, ARM_POT_SWITCH);
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
					if(am->GetSelectedSensorPosition(0)>ARM_POT_SCALE-20)
					{
					iom->Set(ControlMode::Position, pPrefs->GetInt("IOM Out", 50));
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
