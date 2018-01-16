#include "CMotionPointTracker.h"
#include "motionUtils.h"
#include "CActionArc.h"
#include "CActionForward.h"
#include "CMotion.h"

using namespace motion;

CMotionPointTracker::CMotionPointTracker(void)
{
    m_pStateMachine = new CStateMachine<CMotionPointTracker>(this);
    m_pStateMachine->setCurrentState(CPTIdle::getInstance());
}

CMotionPointTracker* CMotionPointTracker::getInstance(void)
{
    static CMotionPointTracker s_instance;
    return &s_instance;
}

void CMotionPointTracker::enter(CMotion* mt)
{
    printf("[ljh] Entering motion PT. \n");
}

void CMotionPointTracker::execute(CMotion* mt)
{
    m_pStateMachine->update();
}

void CMotionPointTracker::exit(CMotion* mt)
{
    setSubMotion(SUBMOTION_IDLE);
    setSubMotionState(SUBMOTION_STATE_NONE);
    printf("[ljh] Exiting motion PT. \n");
}

void CMotionPointTracker::LinearPT(void)
{
    setSubMotion(PT_LPT);

    switch(getlptState())
    {
        /** Rotate to the target direction. */
        case NOT_TRACKING:
            turn2TargPoint();
            break;
        /** Forward with fixed v and calculated distance. */
        case ROTATION_FINISHED:
            trackPoint();
            break;

        case TRACKING_FINISHED:
            mtn->setSubMotionState(PT_FINISHED);
            getFSM()->changeState(CPTIdle::getInstance());
            ///Replaced by changeState(someInstance) later.
            ///todo
            break;

        default:
            printf("[ljh] Invalid point tracking point.\n");
            break;
    }
}

void CMotionPointTracker::ArcPT(void)
{
    setSubMotion(PT_APT);

    switch(mkturns->getActionState())
    {
        case ARC_FINISHED:
            mtn->setSubMotionState(PT_FINISHED);
            printf("[ljh] PT rotation arc finished.\n");
            getFSM()->changeState(CPTIdle::getInstance());
            break;

        case ACTION_NONE:
        case ARC_EXECUTING:
            mkturns->arc(pointTracArcW, arcSide()*180, getDist()/2);
            mtn->setSubMotionState(PT_EXECUTING);
            break;

        case ARC_FAILED:
            mtn->setSubMotionState(PT_FAILED);
            printf("[ljh] PT rotation arc failed.\n");
            ///todo
            break;

        default:
            break;
    }
}

void CMotionPointTracker::turn2TargPoint(void)
{
    switch(mkturns->getActionState())
    {
        case ARC_FINISHED:
            m_lptState = ROTATION_FINISHED;
            mkturns->setActionState(ACTION_NONE);
            printf("[ljh] LPT rotation finished.\n");
            break;

        case ACTION_NONE:
        case ARC_EXECUTING:
            mkturns->arc(pointTracW, m_angle);
            mtn->setSubMotionState(PT_EXECUTING);
            break;

        case ARC_FAILED:
            mtn->setSubMotionState(PT_FAILED);
            printf("[ljh] LPT rotation in place failed.\n");
            ///todo
            break;

        default:
            break;
    }
}

void CMotionPointTracker::trackPoint(void)
{
    switch(mtn->getActionState())
    {
        case FORWARD_FINISHED:
            setlptState(TRACKING_FINISHED);
            printf("[ljh] LPT tracking finished.\n");
            break;

        case ACTION_NONE:
        case FORWARD_EXECUTING:
            fwd->forward(pointTracV, m_dist);
            mtn->setSubMotionState(PT_EXECUTING);
            break;

        case FORWARD_FAILED:
            mtn->setSubMotionState(PT_FAILED);
            printf("[ljh] LPT forward failed.\n");
            ///todo
            break;

        default:

            break;
    }
}

void CMotionPointTracker::calcPhi(void)
{
    double vecX = getMotionParams().PT_loc.x - getCurPose().x;
    double vecY = getMotionParams().PT_loc.y - getCurPose().y;
    TMotionVector2D vecc(vecX, vecY);

    ///double tTargetPhi = atan2(vecY, vecX);
    ///double tAngleDiff = getCurPose().phi - tTargetPhi;
    ///double tNorAbs = motionAbsd(normalizeTheta(tAngleDiff));
    double angle = rad2Deg(acosf(phi2Vec(getCurPose().phi).dotProduct(vecc)));
    ///printf("angle is: %.3f\n", angle);
    ///printf("cphi: %.3f\n",getCurPose().phi);
    double crossProd = phi2Vec(getCurPose().phi).crossProduct(vecc);
    ///printf("tvect x: %.3f, tvect y:%.3f\n", vecX, vecY);
    ///printf("cp: %.3f\n", crossProd);
    m_arcSide = getRotSide(crossProd);
    m_angle = m_arcSide * angle;
}

void CMotionPointTracker::calcDist(void)
{
    m_dist = sqrt((getMotionParams().PT_loc.y - getCurPose().y)
                  * (getMotionParams().PT_loc.y - getCurPose().y)
                  + (getMotionParams().PT_loc.x - getCurPose().x)
                  * (getMotionParams().PT_loc.x - getCurPose().x));
    ///printf("[ljh] dist is: %.3f\n", m_dist);
}

void CMotionPointTracker::reset(void)
{
    resetBase();
    m_lptState = NOT_TRACKING;
    setActionState(ACTION_NONE);
    m_angle = 0;
    m_dist = 0;
}
