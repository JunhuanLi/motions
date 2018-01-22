/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CMotionIdle.cpp
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion idle
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#include "CMotionIdle.h"

using namespace motion;

CMotionIdle::CMotionIdle(void)
{
    ///m_pStateMachine = new CStateMachine<CMotionIdle>(this);
    ///m_pStateMachine->setCurrentState(CIdIdle::getInstance());
}

CMotionIdle* CMotionIdle::getInstance(void)
{
    static CMotionIdle s_instance;
    return &s_instance;
}

void CMotionIdle::enter(CMotion* mt)
{
    setMotion(MOTION_IDLE);
    printf("[ljh] Entering motion IDLE. \n");
}

void CMotionIdle::execute(CMotion* mt)
{
    /// printf("[ljh] Executing motion IDLE. \n");
}

void CMotionIdle::exit(CMotion* mt)
{
    printf("[ljh] Exiting motion IDLE. \n");
}

