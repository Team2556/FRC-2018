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

    // Methods
    void   Init();
    void   SetCommandYaw(float fAngle);
    void   SetCommandYawToCurrent();
    float  GetYaw();
    float  GetYawError();

};

#endif /* SRC_NAVGYRO_H_ */
