/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CPTOwnedStates.cpp
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion point tracker owned state(s)
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#include "CPTOwnedStates.h"

using namespace motion;

/** Point Tracker initial state. */
CPTIdle* CPTIdle::getInstance(void)
{
    static CPTIdle s_instance;
    return &s_instance;
}

void CPTIdle::enter(CMotionPointTracker* pt)
{
    pt->setSubMotion(PT_IDLE);
    printf("[ljh] Entering Point Tracker Idle.\n");
}

void CPTIdle::execute(CMotionPointTracker* pt)
{
    ///printf("[ljh] Executing Point Tracker Idle.\n");
    ///sleep(3);
}

void CPTIdle::exit(CMotionPointTracker* pt)
{
    printf("[ljh] Exiting Point Tracker idle.\n");
}

/** Point Tracker Methods. */
CPTTracking* CPTTracking::getInstance(void)
{
    static CPTTracking s_instance;
    return &s_instance;
}

void CPTTracking::enter(CMotionPointTracker* pt)
{
    pt->getStartPose();
    pt->calcPhi();
    pt->calcDist();
    printf("[ljh] Entering Point Tracker.\n");
}

void CPTTracking::execute(CMotionPointTracker* pt)
{
    if(APT == pt->getMotionParams().PT_mode)
    {
        pt->ArcPT();
    }
    else
    {
        pt->LinearPT();
    }

    /** todos: obs check, etc.*/
}

void CPTTracking::exit(CMotionPointTracker* pt)
{
    printf("[ljh] Exiting Point Tracker.\n");
    pt->resetBase();
    pt->setVelocity(0.0, 0.0);
    ///todo
    pt->reset();
}
