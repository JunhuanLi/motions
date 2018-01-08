#include "CMotionWallFollowing.h"

using namespace motion;

CMotionWallFollowing::CMotionWallFollowing(void)
{
    m_pStateMachine = new CStateMachine<CMotionWallFollowing>(this);
    m_pStateMachine->setCurrentState(CWFWallFol::getInstance());
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
    m_endPhi = 0.0;
}

double CMotionWallFollowing::tuneVelocity(void)
{
    if(getAheadDist() < obsDeceDist)
        return getAheadDist()/obsDeceDist * decelRatio;
    else return 1.0;
}

void CMotionWallFollowing::calcEndPhi(double angRad)
{
    m_endPhi = normalizeTheta(getStartPose().phi + normalizeTheta(angRad, Pi), Pi);
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
    return motionAbsd(rad2Deg(m_endPhi - getCurPose().phi));
}
