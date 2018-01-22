/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file TMotion.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief motion struct
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef TMOTION_H_
#define TMOTION_H_

#include "TMotionPose2D.h"
#include "TMotionParams.h"
#include "motionEnums.h"

namespace motion
{
    /** 
    * Motion shared parameters \n
    *
    */
    struct TMotion
    {
        double linearVelocity;  /**< Linear velocity of the robot.*/
        double angularVelocity;  /**< Angular velocity of the robot.*/
        double sideDist;  /**< The side distance.*/
        double aheadDist;  /**< The ahead distance.*/
        double timeNeeded;  /**< Time needed to finished the task.*/
        double prePhi;  /**< Previous phi.*/
        bool positiveSet;  /**< The positive-direction-set mark*/
        ERotateSide positive;  /**< The positive direction when rotating.*/
        int jumpCount;  /**< Jump count. Jump is defined as the robot rotated -180->180 or 180->-180.*/
        ERotateSide jumpDirection;  /**< Jump direction.*/
        EPTMode ptMode;  /**< The point tracking mode. APT: arc point tracking, LPT: linear point tracking.*/
        bool poseStored;  /**< Pose stored mark.*/
        TMotionPose2D startPose;  /**< Start pose.*/
        TMotionPose2D curPose;  /**< Current pose.*/
        EMotion mot;  /**< Current motion*/
        ESubMotion subMot;  /**< Current sub-motion.*/
        ESubMotionState subMotState;  /**< Current Sub-motion state.*/
        EAction act;  /**< Current action.*/
        EActionState actState;  /**< Current action state.*/
        TMotionParams motParams;  /**< Motion related parameters.*/

        TMotion(void) { init(); }

        inline void init(void)
        {
            linearVelocity = 0;
            angularVelocity = 0;
            sideDist = 0;
            aheadDist = 0;
            timeNeeded = 0;
            prePhi = 0.0;
            positiveSet = false;
            positive = ROTATECCW;
            jumpCount = 0;
            jumpDirection = ROTATECCW;
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
