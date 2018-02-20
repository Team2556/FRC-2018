/*
 * TargetTrack.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: team2556
 */
#include "WPILib.h"
#include <Preferences.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <TargetTrack.h>

bool bMatchPair(cv::Rect Rectangle1, cv::Rect Rectangle2);

// ============================================================================
// TargetTrack
// ============================================================================

// ----------------------------------------------------------------------------
// Constructor / Destructor
// ----------------------------------------------------------------------------

TargetTrack::TargetTrack()
    {
    pPrefs = frc::Preferences::GetInstance();
    UsbCamera = CameraServer::GetInstance()->StartAutomaticCapture();
    fTrackPointX = 0.0;
    fTrackPointY = 0.0;
    }

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void TargetTrack::Init()
    {

    HueLo = pPrefs->GetInt("Target Hue Lo", 0);
    HueHi = pPrefs->GetInt("Target Hue Hi", 255);
    SatLo = pPrefs->GetInt("Target Sat Lo", 0);
    SatHi = pPrefs->GetInt("Target Sat Hi", 255);
    ValLo = pPrefs->GetInt("Target Val Lo", 250);
    ValHi = pPrefs->GetInt("Target Val Hi", 255);

    pPrefs->PutInt("Target Hue Lo", HueLo);
    pPrefs->PutInt("Target Hue Hi", HueHi);
    pPrefs->PutInt("Target Sat Lo", SatLo);
    pPrefs->PutInt("Target Sat Hi", SatHi);
    pPrefs->PutInt("Target Val Lo", ValLo);
    pPrefs->PutInt("Target Val Hi", ValHi);

    iDisplayFrame = pPrefs->GetInt("Display Frame", 0);
    pPrefs->PutInt("Display Frame", iDisplayFrame);

    // Start the camera stuff
	UsbCamera.SetResolution(320, 240);
	UsbCamera.SetFPS(10);

	cvSink   = CameraServer::GetInstance()->GetVideo();
	cvVidOut = CameraServer::GetInstance()->PutVideo("Front Proc", 320, 240);

    }

// ----------------------------------------------------------------------------

void TargetTrack::SetTrackPoint(float fSetTrackPointX, float fSetTrackPointY)
    {
    if      (fSetTrackPointX < -1.0) fTrackPointX = -1.0;
    else if (fSetTrackPointX >  1.0) fTrackPointX =  1.0;
    else                             fTrackPointX =  fSetTrackPointY;

    if      (fSetTrackPointY < -1.0) fTrackPointY = -1.0;
    else if (fSetTrackPointY >  1.0) fTrackPointY =  1.0;
    else                             fTrackPointY =  fSetTrackPointY;
    }


// ----------------------------------------------------------------------------

bool TargetTrack::GetTrackError(float * pfTrackErrorX, float * pfTrackErrorY, float * pfTargetSizeX, float * pfTargetSizeY)
    {
    int iCenterX, iCenterY;

    HueLo = pPrefs->GetInt("Target Hue Lo", 0);
    HueHi = pPrefs->GetInt("Target Hue Hi", 255);
    SatLo = pPrefs->GetInt("Target Sat Lo", 0);
    SatHi = pPrefs->GetInt("Target Sat Hi", 255);
    ValLo = pPrefs->GetInt("Target Val Lo", 250);
    ValHi = pPrefs->GetInt("Target Val Hi", 255);

    iDisplayFrame = pPrefs->GetInt("Display Frame", 0);

    cv::Mat                                 FrameCam;
    cv::Mat                                 FrameHsv;
    cv::Mat                                 FrameThreshold;
//  cv::Mat                                 FrameEdges;
    cv::Mat                                 FrameDraw;

    std::vector< std::vector<cv::Point> >   Contours;
    std::vector< std::vector<cv::Point> >   FiltContours;
//  std::vector< cv::Vect >                 hierarchy;
    std::vector< cv::Rect >                 BoundingRects;
    cv::RNG                                 rng(12345);

    cvSink.GrabFrame(FrameCam);

    // Change image to HSV format
    cv::cvtColor(FrameCam, FrameHsv,  cv::COLOR_BGR2HSV);

    // Change to B/W based on HSV threshold ranges
    cv::inRange(FrameHsv, cv::Scalar(HueLo, SatLo, ValLo), cv::Scalar(HueHi, SatHi, ValHi), FrameThreshold);

    // Detect edges using canny
//  cv::Canny(FrameThreshold, FrameEdges, 100, 200, 3);

    // Find contours
    cv::findContours(FrameThreshold, Contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Filter contours
    for (unsigned int i = 0; i < Contours.size(); i++)
    {
        // Filter based on size
        double dArea = cv::contourArea(Contours[i]);
//              printf(" Area %f ", dArea);
        if (dArea < 100.0)
            continue;

        // Find the upright bounding rectangle
        cv::Rect BRect = cv::boundingRect(Contours[i]);
        float fBRAspect = (float)BRect.height / (float)BRect.width;
//      printf(" R %d %d Aspect %f ", BRect.height, BRect.width, fBRAspect);

        if ((fBRAspect > 3.5) && (fBRAspect < 10.0))
        {
            FiltContours.push_back(Contours[i]);
            BoundingRects.push_back(BRect);
        }
    } // end for all contours

    // Find two bounding rectangles that seem to be a good match
//    printf("%d - ", FiltContours.size());
    int    iRect1, iRect2;
    bool   bMatchFound = false;

    if (FiltContours.size() >= 2)
        {
        unsigned int     iIdx1, iIdx2;
        for (iIdx1 = 0; iIdx1 < FiltContours.size() - 1; iIdx1++)
            for (iIdx2 = iIdx1+1; iIdx2 < FiltContours.size(); iIdx2++)
                {
//                printf("  %d-%d ", iIdx1, iIdx2);
                if (bMatchPair(BoundingRects[iIdx1], BoundingRects[iIdx2]))
                    {
                    iRect1 = iIdx1;
                    iRect2 = iIdx2;
                    bMatchFound = true;
//                    printf("TRUE ");
                    }
                else
                    {
//                    printf("FALSE");
                    }
                }
        } // end if at least two targets

    if (bMatchFound)
        {
        // Find the center of the target in screen coordinates
        // Left = 0  Top = 0
        iCenterX = (BoundingRects[iRect1].x + (BoundingRects[iRect1].width  / 2) +
                    BoundingRects[iRect2].x + (BoundingRects[iRect2].width  / 2)) / 2;
        iCenterY = (BoundingRects[iRect1].y + (BoundingRects[iRect1].height / 2) +
                    BoundingRects[iRect2].y + (BoundingRects[iRect2].height / 2)) / 2;

        // Now scale the position to +1.0 to -1.0
        // Left = -1.0  Right  = +1.0
        // Top  = +1.0  Bottom = -1.0
        *pfTrackErrorX =  (2.0 * (float)iCenterX / FrameThreshold.cols) - 1.0;
        *pfTrackErrorY = -(2.0 * (float)iCenterY / FrameThreshold.rows) + 1.0;

        // Calcualte the target size
        *pfTargetSizeX = fabs((BoundingRects[iRect1].x - BoundingRects[iRect2].x) /  (float)FrameThreshold.cols);
        *pfTargetSizeY = (BoundingRects[iRect1].height + BoundingRects[iRect2].height) /  ((float)FrameThreshold.rows * 2.0);
        }
    else
        {
        // No track so zero out the error just to be safe
        *pfTrackErrorX = 0.0;
        *pfTrackErrorY = 0.0;
        }
    printf("%s %5.2f %5.2f %5.2f %5.2f ", bMatchFound ? "TRACK   " : "NO TRACK", *pfTrackErrorX, *pfTrackErrorY, *pfTargetSizeX, *pfTargetSizeY);

/// Draw contours
    FrameDraw = cv::Mat::zeros(FrameCam.size(), CV_8UC3);
    for (unsigned int i = 0; i < FiltContours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::drawContours(FrameDraw, FiltContours, i, color, 1, 8, cv::noArray(), 0, cv::Point());
    }

printf("\n");

    // Last but not least display a processed camera frame
    switch (iDisplayFrame)
        {
        default :
        case 0 :
            if (!FrameCam.empty()) cvVidOut.PutFrame(FrameCam);
            break;
        case 1 :
            if (!FrameHsv.empty()) cvVidOut.PutFrame(FrameHsv);
            break;
        case 2 :
            if (!FrameThreshold.empty()) cvVidOut.PutFrame(FrameThreshold);
            break;
        case 3 :
            if (!FrameDraw.empty()) cvVidOut.PutFrame(FrameDraw);
            break;
        } // end switch on frame to display

    return bMatchFound;
    } // end Periodic()


// ----------------------------------------------------------------------------
// Utilities
// ----------------------------------------------------------------------------

// Compare two rectangles. Return true if they seem to be a good match pair.

bool bMatchPair(cv::Rect Rectangle1, cv::Rect Rectangle2)
    {
// Find the centers
    //  0,0 Upper Left, 320,240 Lower Right
//    int x1 = Rectangle1.x + (Rectangle1.width  / 2);
    int y1 = Rectangle1.y + (Rectangle1.height / 2);
//    int x2 = Rectangle2.x + (Rectangle2.width  / 2);
    int y2 = Rectangle2.y + (Rectangle2.height / 2);

    // Check verticle alignment
//    printf(" Center Diff %d ", (y1-y2));
    if (fabs(y1 - y2) > 10)
         return false;

    // Check relative areas
    float fTargetArea1 = Rectangle1.width * Rectangle1.height;
    float fTargetArea2 = Rectangle2.width * Rectangle2.height;
    float fAreaRatio   = fTargetArea1 / fTargetArea2;

//  printf(" Area %f %f %5.2f ", fTargetArea1, fTargetArea2, fAreaRatio);
    if ((fAreaRatio < 0.5) || (fAreaRatio > 2.0))
        return false;

//    printf("  X,Y=%d,%d XxY=%dx%d   X,Y=%d,%d XxY=%dx%d",
//            Rectangle1.x, Rectangle1.y, Rectangle1.width, Rectangle1.height,
//            Rectangle2.x, Rectangle2.y, Rectangle2.width, Rectangle2.height);
//    printf(" Center Diff %d ", (y1-y2));
//    printf(" Area %f %f %5.2f ", fTargetArea1, fTargetArea2, fAreaRatio);

    return true;
    }
