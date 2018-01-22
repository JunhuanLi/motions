/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CActionBackward.cpp
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Backward action
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#include "CActionBackward.h"
using namespace motion;

CActionBackward* CActionBackward::getInstance(void)
{
    static CActionBackward s_instance;
    return &s_instance;
}

void CActionBackward::backward(double v, double distance)
{
    static mrpt::utils::CTicTac	s_tictac;

    /// Store current information and calculate information needed.
    if(!getPoseStored())
    {
        reset();
        setAction(BACKWARD);
        storeStartPose();
        calcEndPose(-motionAbsd(distance));
        calcTimeNeeded(v, distance);

        s_tictac.Tic();  /// start the timer
    }

    /// Timeout check.
    if(s_tictac.Tac() > getTimeNeeded() + 1)
    {
        ///throw exceptions
        ///assert(!"Time out when backwarding.");
        printf("[ljh] Backward timeOut.\n");
        setVelocity(0.0, 0.0);
        setPoseStored(false);
        setActionState(BACKWARD_FAILED);
        return;
    }

    /// Finish condition check.
    if(calcRem() <= locAccuracy)
    {
        printf("[ljh] Backward finished.\n");
        setVelocity(0.0, 0.0);
        setPoseStored(false);
        setActionState(BACKWARD_FINISHED);
        setAction(STOP);
    }
    else
    {
        setVelocity(-motionAbsd(v), 0.0);
        setActionState(BACKWARD_EXECUTING);
    }
}

void CActionBackward::calcEndPose(double distance)
{
    TMotionPose2D absPose = frameRot(distance, 0, -getCurPose().phi);
    TMotionPose2D startPose = getStartPose();
    m_endPose = startPose + absPose;
}

double CActionBackward::calcRem()
{
    double curDistX = (getCurPose().x - m_endPose.x)
                      * (getCurPose().x - m_endPose.x);
    double curDistY = (getCurPose().y - m_endPose.y)
                      * (getCurPose().y - m_endPose.y);
    return sqrt(curDistX + curDistY);
}

void CActionBackward::reset()
{
    resetBase();

    m_endPose.reset();
}

