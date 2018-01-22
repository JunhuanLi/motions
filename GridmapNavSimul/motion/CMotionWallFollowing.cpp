/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CMotionWallFollowing.cpp
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion wall following
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#include "CMotionWallFollowing.h"

using namespace motion;

CMotionWallFollowing::CMotionWallFollowing(void)
{
    m_pStateMachine = new CStateMachine<CMotionWallFollowing>(this);
    m_pStateMachine->setCurrentState(CWFWallFol::getInstance());
    wfController = TMotionPIDCon(wfPID_P, wfPID_I, wfPID_D);
}

CMotionWallFollowing* CMotionWallFollowing::getInstance(void)
{
    static CMotionWallFollowing s_instance;
    return &s_instance;
}

void CMotionWallFollowing::enter(CMotion* mt)
{
    ///setMotion(WALL_FOLLOWING);
    printf("[ljh] Entering motion WF. \n");
}

void CMotionWallFollowing::execute(CMotion* mt)
{
    m_pStateMachine->update();
}

void CMotionWallFollowing::exit(CMotion* mt)
{
    printf("[ljh] Exiting motion WF. \n");
}

void CMotionWallFollowing::reset()
{
    resetBase();
    m_andState = NOT_SATISFIED;
}

double CMotionWallFollowing::tuneVelocity(void)
{
    if(getAheadDist() < obsDeceDist)
        return getAheadDist()/obsDeceDist * decelRatio;
    else return 1.0;
}

bool CMotionWallFollowing::distReached(void)
{
    if(getAheadDist() >= wallFolResumeDist)
    {
        printf("[ljh] Dist %f meter satisfied.\n", wallFolResumeDist);
        return true;
    }
    return false;
}

bool CMotionWallFollowing::angNDistReached(void)
{
    if(!m_andState)
    {
        if(calcRemAngle() <= angAccuracy)
        {
            printf("[ljh] AND angle %.3f deg satisfied.\n", wallFolResumeANDAngle);
            m_andState = ANGLE_SATISFIED;
        }
    }

    if(ANGLE_SATISFIED == m_andState)
    {
        if(getAheadDist() >= obsStopDist)
        {
            printf("[ljh] AND dist %f meter satisfied.\n", obsStopDist);
            m_andState = DIST_SATISFIED;
        }
    }

    if(DIST_SATISFIED == m_andState)
    {
        m_andState = NOT_SATISFIED;
        return true;
    }

    return false;
}

double CMotionWallFollowing::calcRemAngle(void)
{
    double myCurPhi = getCurPose().phi + twoPi * getJumpDir() * getJumpCount();
    double phiRotated = myCurPhi - getStartPose().phi;
    double angRem = wallFolResumeANDAngle - rad2Deg(phiRotated);
    ///printf("remain angle: %.3f\n", angRem);
    return angRem;
    ///return motionAbsd(rad2Deg(m_endPhi - getCurPose().phi));
}
