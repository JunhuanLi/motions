#ifndef MOTIONENUMS_H_
#define MOTIONENUMS_H_

#include "TMotionPose2D.h"

namespace motion
{
    enum EMotion
    {
        MOTION_IDLE = 0,  /** Inital motion. */
        WALL_FOLLOWING,  /**< Wall following. */
        POINT_TRACKING  /**< Point tracking. */
    };

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

    enum ESubMotionState
    {
        SUBMOTION_STATE_NONE = 0,  /**< Initial state. */
        PT_FAILED,  /**< Point tracking failed. */
        PT_FINISHED,  /**< Point tracking finished. */
        PT_EXECUTING  /**< Point tracking executing. */
    };

    enum EAction
    {
        STOP = 0,  /**< Stop. */
        FORWARD,  /**< Forward. */
        BACKWARD,  /**< Backward. */
        ARC  /**< Arc. */
    };

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

    enum EPTMode
    {
        LPT = 0,  /**< Linear point tracking. */
        APT  /**< Arc point tracking. */
    };

    enum EPTBumpMode
    {
        FRAGILE = 0,  /**< Assert PT failed if ONE bump occurred. */
        STRONG  /**< Assert PT failed if SEVERAL bumps occurred. */
    };

    enum ERotateSide
    {
        ROTATECCW = 1,  /**< Rotate counterclockwise. */
        ROTATECW  = -1  /**< Rotate clockwise. */
    };

    enum EAngNDistReachedState
    {
        NOT_SATISFIED = 0,  /**< Initial state. */
        ANGLE_SATISFIED,  /**< Angle condition satisfied. */
        DIST_SATISFIED  /**< Distance condition satisfied. */
    };

    enum ELinearPTState
    {
        NOT_TRACKING = 0,  /**< Initial state. */
        ROTATION_FINISHED,  /**< First stage: rotation finished. */
        TRACKING_FINISHED  /**< Second stage: Tracking finished. */
    };
}

#endif //MOTIONENUMS_H_

