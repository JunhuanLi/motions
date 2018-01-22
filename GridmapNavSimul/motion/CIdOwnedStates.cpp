/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CIdOwnedStates.cpp
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion idle owned state(s) 
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#include "CIdOwnedStates.h"

using namespace motion;

/** Idle Methods. */
CIdIdle* CIdIdle::getInstance(void)
{
    static CIdIdle s_instance;
    return &s_instance;
}

void CIdIdle::enter(CMotionIdle* id)
{

    printf("[ljh] Entering CIdIdle.\n");
}

void CIdIdle::execute(CMotionIdle* id)
{
    /// Do nothing.
    ///printf("[ljh] Executing CIdIdle.\n");
}

void CIdIdle::exit(CMotionIdle* id)
{
    printf("[ljh] Exiting CIdIdle.\n");
}
