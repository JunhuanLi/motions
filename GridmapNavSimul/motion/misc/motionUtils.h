#ifndef MOTIONUTILS_H_
#define MOTIONUTILS_H_

#include "motionEnums.h"
#include "TMotionPose2D.h"
#include "TMotionVector2D.h"
#include <cstdio>

namespace motion
{
    /** Normalize the give theta to the given range.
     * Generally the range is 180 degree or Pi, then this function will
     * correspondingly normal the theta value into -180~180 or -Pi~Pi.
     * @param theta Angle need normalize.
     * @param range Target range with default value 180 degree.
     * @return Normalized angle.
      */
    inline double normalizeTheta(double theta, double range = 180);
    /** Translate angle in degree to angle in rad. */
    inline double deg2Rad(double angDeg);
    /** Translate angle in rad to angle in degree. */
    inline double rad2Deg(double angRad);

    /** Micros*/

    /** Constants */
    const double Pi = 3.14159;
    const double halfPi = Pi / 2;
    const double twoPi = 2 * Pi;
    const double deg2RadRatio = Pi / 180;
    const double rad2DegRatio = 180 / Pi;
    const double angAccuracy = 1;  /**< Angle accuracy in degree. */
    const double locAccuracy = 0.1;  /**< Location accuracy in meter. */

    /** Wall Following parameters.*/
    const double infraredRayAngle = 0;  /**< Angle between infrared ray and the direction which prependicular to the robot heading direction. It's a positive number between 0~90 degree*/
    const double winLen = 5;  /**< The length of window that pick part of laser data. */
    const double wallFolV = 0.3;  /**< Linear velocity(m/s) in following wall. */
    const double wallFolW = deg2Rad(90);  /**< Angular velocity(rad/s) when deviated in following wall. */
    const double wallFolSideDist = 0.5;  /**< The side distance that robot need to keep. */
    const double obsStopDist = 0.5;  /**< The distance that robot need to stop. */
    const double wallFolResumeDist = 3;  /**< Resume-wall-following-mode distance. */
    const double wallFolResumeANDAngle = 90;  /**< Resume-wall-following-mode angle(degree)*/
    const double wallFolObsW = deg2Rad(60);  /**< Angular velocity when obstacles detected. */
    const double wallFolObsR = 0.0;  /**< Rotation radius when obstacles detected. */
    const double wallFolRidgeW = deg2Rad(60);  /**< Angular velocity when edges detected. */
    const double wallFolRidgeR = 0.01;  /**< Rotation radius when edges detected. */
    const double decelRatio = 0.5;  /**< Deceleration ratio(0~1). */
    const double obsDeceDist = obsStopDist * 1.5;  /**< The distance that robot need to start decelerate. */
    const double distErrTolerance = 0.01;  /**< Wall following error tolerance.*/
    const double maxAllowedSideDist = 1.5;  /**< Maximum allowed side distance. If larger than this value, it means edge reached.*/

    /** Point Tracker parameters. */
    const double pointTracW = deg2Rad(40);  /**< Point Tracking rotation velocity. */
    const double pointTracV = 1;  /**< Point Tracking linear velocity. */
    const double pointTracArcW = deg2Rad(20);  /**< Point Tracking arc angular velocity. */

    /** Functions */

    inline double normalizeTheta(double theta, double range)
    {
        while(theta > range)
        {
            theta -= 2*range;
        }

        while(theta < -range)
        {
            theta += 2*range;
        }
        return theta;
    }

    inline double deg2Rad(double angDeg)
    {
        return angDeg * deg2RadRatio;
    }

    inline double rad2Deg(double angRad)
    {
        return angRad * rad2DegRatio;
    }

    inline double motionAbsd(double val)
    {
        if(val < 0) val = -val;
        return val;
    }

    inline ERotateSide getRotSide(double angDeg)
    {
        if(angDeg < 0) return ROTATECW;
        else return ROTATECCW;
    }

    inline TMotionVector2D phi2Vec(double angRad)
    {
        TMotionVector2D vec(cos(angRad),sin(angRad));
        vec.normVec();
        return vec;
    }


}
#endif /** MOTIONUTILS_H_ */
