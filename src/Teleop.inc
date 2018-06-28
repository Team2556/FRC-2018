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
	int iomInPref = pPrefs->GetInt("IOM In", 585);
	pPrefs->PutInt("IOM In", iomInPref);
	int iomOutPref = pPrefs->GetInt("IOM Out", 54);
	pPrefs->PutInt("IOM Out", iomOutPref);
	int iomMidPref = pPrefs->GetInt("IOM Mid", 360);
	pPrefs->PutInt("IOM Mid", iomMidPref);
    }


// ----------------------------------------------------------------------------

void TeleopPeriodic() {
	pNavGyro->UpdateValues();
//	float  fTrackErrorX, fTrackErrorY;
//    float  fTargetSizeX, fTargetSizeY;
//    bool bTrackLock;
   // bTrackLock = pTrack->GetTrackError(&fTrackErrorX, &fTrackErrorY, &fTargetSizeX, &fTargetSizeY);
//    SmartDashboard::PutNumber("Target Size", fTargetSizeX);
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
//		float PsiValue;
//		double Vout = AnalogIn2->GetVoltage() ;
//		PsiValue = 250*(Vout/2.09384615)-25;
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

	//climbSolenoid->Set(pclXbox2->GetYButton() ? frc::DoubleSolenoid::Value::kReverse : frc::DoubleSolenoid::Value::kOff);
	pClimbTrigger->Set(pclXbox2->GetStartButton() ? frc::Relay::Value::kReverse : frc::Relay::Value::kOff);
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
	armPot= armPot+5;
    }
    else if(pclXbox2->GetTriggerAxis(frc::XboxController::kLeftHand)> 0.1){
    	armPot= armPot-5;
    }
    else
    {
    	armPot = am->GetSelectedSensorPosition(0);
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
    	iom->Set(ControlMode::Position, pPrefs->GetInt("IOM In", 585));
    	/*if(iom->GetSelectedSensorPosition(0)<= pPrefs->GetInt("IOM In",600))
    	{
    		am->Set(ControlMode::PercentOutput, 0);
    	}*/
    }

    else if( (am->GetSelectedSensorPosition(0) < pPrefs->GetInt("Upper Grab Height", 1023) &&  am->GetSelectedSensorPosition(0) > pPrefs->GetInt("Lower Grab Height", 0))|| am->GetSelectedSensorPosition(0) > pPrefs->GetInt("Upper Scale Height", 1023))
    {
    	 if (pclXbox2->GetBumper(frc::XboxController::kLeftHand) && pclXbox2->GetBumper(frc::XboxController::kRightHand) )
    	        {
    	        	iom->Set(ControlMode::Position, pPrefs->GetInt("IOM Mid", 360));
    	        }
    	    else if(pclXbox2->GetBumper(frc::XboxController::kRightHand))
    	    {
    	    	iom->Set(ControlMode::Position, pPrefs->GetInt("IOM In", 585));
    	    }
    	    else if(pclXbox2->GetBumper(frc::XboxController::kLeftHand))
    	    {
    	    	iom->Set(ControlMode::Position, pPrefs->GetInt("IOM Out", 54));
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
