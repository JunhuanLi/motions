#ifndef TMOTION_H_
#define TMOTION_H_

#include "TMotionPose2D.h"
#include "TMotionParams.h"
#include "motionEnums.h"

namespace motion
{
    struct TMotion
    {
        double linearVelocity;
        double angularVelocity;
        double sideDist;
        double aheadDist;
        double timeNeeded;
        EPTMode ptMode;
        bool poseStored;
        TMotionPose2D startPose;
        TMotionPose2D curPose;
        EMotion mot;
        ESubMotion subMot;
        ESubMotionState subMotState;
        EAction act;
        EActionState actState;
        TMotionParams motParams;

        TMotion(void) { init(); }

        inline void init(void)
        {
            linearVelocity = 0;
            angularVelocity = 0;
            sideDist = 0;
            aheadDist = 0;
            timeNeeded = 0;
            poseStored = false;
            startPose.reset();
            curPose.reset();

            mot = MOTION_IDLE;
            subMot = SUBMOTION_IDLE;
            subMotState = SUBMOTION_STATE_NONE;
            act = STOP;
            actState = ACTION_NONE;

            motParams.reset();
        }
    };
}
#endif /** TMOTION_H_ */
