/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file motionEnums.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Enums defination
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef MOTIONENUMS_H_
#define MOTIONENUMS_H_

#include "TMotionPose2D.h"

namespace motion
{
   /** 
    * Motion name collection \n
    *
    */
    enum EMotion
    {
        MOTION_IDLE = 0,  /**< Inital motion. */
        WALL_FOLLOWING,  /**< Wall following. */
        POINT_TRACKING  /**< Point tracking. */
    };
   /** 
    * Sub-motion name collection \n
    *
    */
    enum ESubMotion
    {
        SUBMOTION_IDLE = 0,  /**< Initial submotion. */
        WF_OBS,  /**< Obstacle avoidance in wall following motion. */
        WF_RIDGE,  /**< Ridge tracking in wall following motion. */
        WF_FOLLOWING,  /**< wall following in wall following motion. */
        PT_IDLE,  /**< Change to PT_IDLE automatically if point tracking finished. */
        PT_LPT,  /**< Linear point tracking. */
        PT_APT  /**< Arc point tracking. */
    };
   /** 
    * Sub-motion state collection \n
    *
    */
    enum ESubMotionState
    {
        SUBMOTION_STATE_NONE = 0,  /**< Initial state. */
        PT_FAILED,  /**< Point tracking failed. */
        PT_FINISHED,  /**< Point tracking finished. */
        PT_EXECUTING  /**< Point tracking executing. */
    };
   /** 
    * Action name collection \n
    *
    */
    enum EAction
    {
        STOP = 0,  /**< Stop. */
        FORWARD,  /**< Forward. */
        BACKWARD,  /**< Backward. */
        ARC  /**< Arc. */
    };
   /** 
    * Action state collection \n
    *
    */
    enum EActionState
    {
        ACTION_NONE = 0,  /**< Initial state. */
        FORWARD_EXECUTING,  /**< The current forward order is executing. */
        FORWARD_FINISHED,  /**< The forward order is successed. */
        FORWARD_FAILED,  /**< The forward order is failed. */
        BACKWARD_EXECUTING,  /**< The current backward order is executing. */
        BACKWARD_FINISHED,  /**< The backward order is successed. */
        BACKWARD_FAILED,  /**< The backward order is failed. */
        ARC_EXECUTING,  /**< The current arc order is executing. */
        ARC_FINISHED,  /**< The arc order is successed. */
        ARC_FAILED  /**< The arc order is failed. */
    };
   /** 
    * Point tracking mode \n
    *
    */
    enum EPTMode
    {
        LPT = 0,  /**< Linear point tracking. */
        APT  /**< Arc point tracking. */
    };
   /** 
    * Point tracking bump mode \n
    *
    */
    enum EPTBumpMode
    {
        FRAGILE = 0,  /**< Assert PT failed if ONE bump occurred. */
        STRONG  /**< Assert PT failed if SEVERAL bumps occurred. */
    };
   /** 
    * Rotate side \n
    *
    */
    enum ERotateSide
    {
        ROTATECCW = 1,  /**< Rotate counterclockwise. */
        ROTATECW  = -1  /**< Rotate clockwise. */
    };
   /** 
    * Angle and distance reached state collection \n
    *
    */
    enum EAngNDistReachedState
    {
        NOT_SATISFIED = 0,  /**< Initial state. */
        ANGLE_SATISFIED,  /**< Angle condition satisfied. */
        DIST_SATISFIED  /**< Distance condition satisfied. */
    };
   /** 
    * Linear point tracking state collection \n
    *
    */
    enum ELinearPTState
    {
        NOT_TRACKING = 0,  /**< Initial state. */
        ROTATION_FINISHED,  /**< First stage: rotation finished. */
        TRACKING_FINISHED  /**< Second stage: Tracking finished. */
    };
}

#endif /** MOTIONENUMS_H_ */

