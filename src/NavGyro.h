/*
 * NavGyro.h
 *
 *  Created on: Feb 1, 2018
 *      Author: team2556
 */

#ifndef SRC_NAVGYRO_H_
#define SRC_NAVGYRO_H_

#ifdef NAVX
#include <AHRS.h>
#endif

#ifdef ADXRS_GYRO
#include <ADXRS450_Gyro.h>
#endif

class NavGyro
{
public:
    // Constructor / Destructor
    NavGyro();
    virtual ~NavGyro();

    // Data
#ifdef NAVX
    AHRS *  		pNavX;
#endif
#ifdef ADXRS_GYRO
    ADXRS450_Gyro *	pADXRS;
#endif

    float		fGyroCommandYaw;
    bool		bPresetTurning;

    // Methods
    void   Init();
    void   UpdateValues();
    void   SetCommandYaw(float fAngle);
    void   SetCommandYawToCurrent();
    float  GetYaw();
    float  GetYawError();
    float  CorrectRotate(float fRotateLess);
    float  GetRotate();
    float	GetDisplacemetX();
    float	GetDisplacemetY();
    float	GetDisplacemetZ();

};

#endif /* SRC_NAVGYRO_H_ */
