#include <apps/GridmapNavSimul/motion/action/CActionForward.h>
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
    /** If ahead distance is small enough, then go obstacle avoidance. */
    if(wf->getAheadDist() <= obsStopDist)
    {
        wf->getFSM()->changeState(CWFObsAvoid::getInstance());
        return;
    }
    /** If side distance have big jump, then go ridge tracking. */
    if(wf->getSideDist() >= maxAllowedSideDist)
    {
        wf->getFSM()->changeState(CWFRidgeTrack::getInstance());
        return;
    }

    /** Wall following. */
    double v = wallFol->getMotionParams().WF_V;
    v *= wf->tuneVelocity();

    if(wf->getSideDist() < wallFol->getMotionParams().WF_dist - distErrTolerance)
    {
        wf->setVelocity(v, wallFolW);  ///todo w!=constant
    }
    else if(wf->getSideDist() > wallFol->getMotionParams().WF_dist + distErrTolerance)
    {
        wf->setVelocity(v, -wallFolW);
    }
    else
    {
        wf->setVelocity(v, 0.0);
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
                ///todo
            }
    }
}

void CWFRidgeTrack::exit(CMotionWallFollowing* wf)
{
    wf->setVelocity(0.0, 0.0);
    wf->reset();
}

