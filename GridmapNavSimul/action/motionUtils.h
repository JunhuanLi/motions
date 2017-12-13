#ifndef MOTIONUTILS_H_
#define MOTIONUTILS_H_

#include "motionEnums.h"

namespace motion
{
    /** Constants */
    const double Pi = 3.14159;
    const double twoPi = 2 * Pi;
    const double deg2RadRatio = Pi / 180;
    const double rad2DegRatio = 180 / Pi;


    /** Functions */
    inline void normlizeTheta(double& theta, double range = 180)
    {
        while(theta > range)
        {
            theta -= range;
        }

        while(theta < -range)
        {
            theta += range;
        }
    }

    inline double deg2Rad(double angDegree)
    {
        return angDegree * deg2RadRatio;
    }

    inline double rad2Deg(double angRad)
    {
        return angRad * rad2DegRatio;
    }

    inline ERotateOrientation getRotateOrientation(double angDegree)
    {
        if(angDegree < 0) return ROTATECW;
        else return ROTATECCW;
    }
}
#endif // MOTIONUTILS_H_
