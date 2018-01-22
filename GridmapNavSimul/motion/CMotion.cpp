/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CMotion.cpp
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion (owner of any motion)
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#include "CMotion.h"

using namespace motion;

CMotion::CMotion(void)
{
    m_pStateMachine = new CStateMachine<CMotion>(this);
    m_pStateMachine->setCurrentState(CMotionIdle::getInstance());
}

CMotion* CMotion::getInstance(void)
{
    static CMotion s_instance;
    return &s_instance;
}

void CMotion::update(void)
{
    /// Check bump and lift later here.

    m_pStateMachine->update();
}

void CMotion::reset()
{

}
