/*
 * NavGyro.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: team2556
 */

#include "WPILib.h"

#include "RobotMap.h"

#ifdef NAVX
#include <AHRS.h>
#endif

#ifdef ADXRS_GYRO
#include <ADXRS450_Gyro.h>
#endif

#include <NavGyro.h>

float fNormalizeAngle360(float fAngle);
float fNormalizeAngle180(float fAngle);

// ============================================================================
// NavGyro
// ============================================================================

// ----------------------------------------------------------------------------
// Constructor / Destructor
// ----------------------------------------------------------------------------

NavGyro::NavGyro()
    {
#ifdef NAVX
    // Make the NavX control object
    pNavX = new AHRS(SPI::Port::kMXP);
#endif

#ifdef ADXRS_GYRO
    pADXRS = new ADXRS450_Gyro(SPI::Port::kOnboardCS0);
#endif

    }


// ----------------------------------------------------------------------------

NavGyro::~NavGyro()
    {
	// TODO Auto-generated destructor stub
    }

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void NavGyro::Init()
    {
    // Get the initial starting angle
#ifdef NAVX
    fGyroCommandYaw = pNavX->GetYaw();
#endif
#ifdef ADXRS_GYRO
//    pADXRS->Calibrate();
    fGyroCommandYaw = pADXRS->GetAngle();
#endif
    }


// ----------------------------------------------------------------------------

void NavGyro::SetCommandYaw(float fAngle)
    {
    fGyroCommandYaw = fNormalizeAngle360(fAngle);
    }


// ----------------------------------------------------------------------------

void NavGyro::SetCommandYawToCurrent()
    {
#ifdef NAVX
    fGyroCommandYaw = pNavX->GetYaw();
#endif
#ifdef ADXRS_GYRO
    fGyroCommandYaw = pADXRS->GetAngle();
#endif
    }


// ----------------------------------------------------------------------------

float NavGyro::GetYaw()
    {
#if defined(NAVX)
    return pNavX->GetYaw();
#elif defined(ADXRS_GYRO)
    return pADXRS->GetAngle();
#else
    return 0.0;
#endif
    }


// ----------------------------------------------------------------------------

float  NavGyro::GetYawError()
    {
    return -fNormalizeAngle180(GetYaw() - fGyroCommandYaw);
    }


//-----------------------------------------------------------------------------
float  NavGyro::CorrectRotate(float fRotateLess)
{
	if(fRotateLess >  0.5)
	{
		fRotateLess =  0.5;
	}
	if(fRotateLess < -0.5)
	{
		fRotateLess = -0.5;
	}
	return fRotateLess;
}
float  NavGyro::GetRotate()
	{
		float YawError = this->GetYawError() *0.05;
		YawError = this->CorrectRotate(YawError);
		return YawError;
	}

// ----------------------------------------------------------------------------
// Utilities
// ----------------------------------------------------------------------------

// Normalize fAngle range from 0.0 to 360.0

float fNormalizeAngle360(float fAngle)
    {
    while (fAngle <    0.0) fAngle += 360.0;
    while (fAngle >= 360.0) fAngle -= 360.0;
    return fAngle;
    }


// ----------------------------------------------------------------------------

// Normalize fAngle range from +180.0 to -180.0

float fNormalizeAngle180(float fAngle)
    {
    while (fAngle <  -180.0) fAngle += 360.0;
    while (fAngle >=  180.0) fAngle -= 360.0;
    return fAngle;
    }

