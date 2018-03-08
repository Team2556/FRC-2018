/*
 * TargetTrack.h
 *
 *  Created on: Feb 13, 2018
 *      Author: team2556
 */

#ifndef SRC_TARGETTRACK_H_
#define SRC_TARGETTRACK_H_

class TargetTrack
    {
    // Constructor / Destructor
public:
    TargetTrack();

    // Data
    frc::Preferences *  pPrefs;
    cs::UsbCamera       UsbCamera;
    cs::UsbCamera       UsbCamera1;
    cs::CvSink          cvSink;
    cs::CvSource        cvVidOut;

    int                 HueLo, HueHi;
    int		            SatLo, SatHi;
    int		            ValLo, ValHi;

    int                 iDisplayFrame;

    float               fTrackPointX;
    float               fTrackPointY;

    // Methods
    void    Init(void);
    void    SetTrackPoint(float fTrackPointX, float fTrackPointY);
    bool    GetTrackError(float * pfTrackErrorX, float * pfTrackErrorY, float * pfTargetSizeX, float * pfTargetSizeY);

};

#endif /* SRC_TARGETTRACK_H_ */
