/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CWFOwnedStates.cpp
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion wall following owned state(s)
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#include "action/CActionForward.h"
#include "CWFOwnedStates.h"
#include "CActionArc.h"
using namespace motion;

/** Wall Following Methods. */
CWFWallFol* CWFWallFol::getInstance(void)
{
    static CWFWallFol s_instance;
    return &s_instance;
}

void CWFWallFol::enter(CMotionWallFollowing* wf)
{
    wf->setSubMotion(WF_FOLLOWING);
    printf("[ljh] Wall following.\n");
}

void CWFWallFol::execute(CMotionWallFollowing* wf)
{
    /// If ahead distance is small enough, then go obstacle avoidance.
    if(wf->getAheadDist() <= obsStopDist)
    {
        wf->getFSM()->changeState(CWFObsAvoid::getInstance());
        return;
    }
    /// If side distance have big jump, then go ridge tracking.
    if(wf->getSideDist() >= maxAllowedSideDist)
    {
        wf->getFSM()->changeState(CWFRidgeTrack::getInstance());
        return;
    }

    /// Wall following.
    double v = wallFol->getMotionParams().WF_V;
    double distErr = wallFol->getMotionParams().WF_dist - wf->getSideDist();

    v *= wf->tuneVelocity();

    if(motionAbsd(distErr) < distErrTolerance)
    {
        wf->setVelocity(v, 0.0);
    }
    else
    {
        double ctrlOutput = wallFol->getWFController().pid(distErr);
        ///printf("co = %.3f\n", ctrlOutput);
        wf->setVelocity(v, ctrlOutput);
    }
}

void CWFWallFol::exit(CMotionWallFollowing* wf)
{
    wf->setVelocity(0.0, 0.0);
    wf->reset();
}

/** Obstacle Avoidance Methods. */
CWFObsAvoid* CWFObsAvoid::getInstance(void)
{
    static CWFObsAvoid s_instance;
    return &s_instance;
}

void CWFObsAvoid::enter(CMotionWallFollowing* wf)
{
    wf->setSubMotion(WF_OBS);
    printf("[ljh] Obs detected.\n");
    wf->storeStartPose();
}

void CWFObsAvoid::execute(CMotionWallFollowing* wf)
{
    if(wf->angNDistReached() || wf->distReached())
    {
        printf("[ljh] WF_Obs avoidance finished.\n");
        wf->getFSM()->changeState(CWFWallFol::getInstance());
    }
    else
    {
        mkturns->arc(wallFolObsW, 360, wallFolObsR);
        if(ARC_EXECUTING == mkturns->getActionState())
        {
            wallFol->setVelocity(mkturns->getLinearVelocity(), mkturns->getAngularVelocity());
        }
        else
        {
            ///todo
        }
    }
}

void CWFObsAvoid::exit(CMotionWallFollowing* wf)
{
    wf->setVelocity(0.0, 0.0);
    wf->reset();
}

/** Ridge Tracking Methods. */
CWFRidgeTrack* CWFRidgeTrack::getInstance(void)
{
    static CWFRidgeTrack s_instance;
    return &s_instance;
}

void CWFRidgeTrack::enter(CMotionWallFollowing* wf)
{
    wf->setSubMotion(WF_RIDGE);
    printf("[ljh] Ridge detected.\n");
}

void CWFRidgeTrack::execute(CMotionWallFollowing* wf)
{
    if(wf->getSideDist() <= wallFol->getMotionParams().WF_dist + distErrTolerance + wallFolRidgeR)
    {
        printf("[ljh] WF_Ridge tracking finished.\n");
        wf->getFSM()->changeState(CWFWallFol::getInstance());
    }
    else
    {
            mkturns->arc(wallFolRidgeW, -360, wallFolRidgeR);
            if(ARC_EXECUTING == mkturns->getActionState())
            {
                wallFol->setVelocity(wallFol->getLinearVelocity(), wallFol->getAngularVelocity());
            }
            else
            {
                ///todo exceptions
            }
    }
}

void CWFRidgeTrack::exit(CMotionWallFollowing* wf)
{
    wf->setVelocity(0.0, 0.0);
    wf->reset();
}

