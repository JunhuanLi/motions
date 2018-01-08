#include "CActionArc.h"

using namespace motion;

CActionArc* CActionArc::getInstance(void)
{
    static CActionArc s_instance;
    return &s_instance;
}

void CActionArc::arc(double w, double angDeg, double radius)
{
    static mrpt::utils::CTicTac	s_tictac;

    /** how to avoid multiple tasks? */

    /** Store current information and calculate information needed*/
    if(!getPoseStored())
    {
        reset();
        setAction(ARC);
        startUp(angDeg);
        storeStartPose();
        calcTimeNeeded(w, deg2Rad(angDeg));
        calcInnerStrLen(angDeg, radius);
        m_linVel = calcLinVel(angDeg, radius);
        calcEndPose(angDeg);

        s_tictac.Tic();  /// Start the timer
    }

    /** If start up is enabled, update if the vehicle get started everytime. */
    if(getStartUp())
        updateStartUp();

    /** Timeout check.*/
    if(s_tictac.Tac() > getTimeNeeded() + 1)
    {
        ///throw exceptions
        ///assert(!"Time out when making Arc move.");
        setVelocity(0.0, 0.0);
        setPoseStored(false);
        setActionState(ARC_FAILED);
    }

    /** Finish condition check.*/
    if(calcRem() <= angAccuracy && !getStartUp())
    {
        printf("[ljh] Direction reached.\n");
        setVelocity(0.0, 0.0);
        if(poseVerified())
        {
            printf("[ljh] Pose verify passed.\n");
            setPoseStored(false);
            setActionState(ARC_FINISHED);
            setAction(STOP);
        }
        else
        {
            printf("[ljh] Pose verify not passed.\n");
            ///throw exceptions
            ///assert(!"Direction reached but can't reach the appropriate location.");
            setPoseStored(false);
            setActionState(ARC_FAILED);
        }
    }
    else
    {
        ///w = m_controller.pid(calcRem());
        setVelocity(motionAbsd(m_linVel), motionAbsd(w) * getRotSide(angDeg));
        setActionState(ARC_EXECUTING);
    }
}

void CActionArc::calcEndPose(double angDeg)
{
    double angRad = deg2Rad(angDeg);
    double yawRad = calcYaw(angRad);
    TMotionPose2D absPose = frameRot(m_innerStrLen * cos(yawRad),
                                     m_innerStrLen * sin(yawRad),
                                     -getCurPose().phi);
    m_endPose = getStartPose() + absPose;
    m_endPose.phi = normalizeTheta(getStartPose().phi + normalizeTheta(angRad, Pi), Pi);
}

double CActionArc::calcRem(void)
{
    return motionAbsd(rad2Deg(m_endPose.phi - getCurPose().phi));
}

bool CActionArc::poseVerified(void)
{
    double tDist = sqrt((getCurPose().x - m_endPose.x)
                           * (getCurPose().x - m_endPose.x)
                           + (getCurPose().y - m_endPose.y)
                           * (getCurPose().y - m_endPose.y));

    if(tDist <= 1) return true;
    else return false;
}

void CActionArc::startUp(double angDeg)
{
    if(motionAbsd(rad2Deg(twoPi) - motionAbsd(angDeg)) <= angAccuracy * 3)
    {
        setStartUp(true);
    }
    else
    {
        setStartUp(false);
    }
}

void CActionArc::updateStartUp(void)
{
    if(motionAbsd(rad2Deg(getCurPose().phi - getStartPose().phi)) > angAccuracy * 10)
    {
        setStartUp(false);
    }
}

double CActionArc::calcLinVel(double angDeg, double radius)
{
    angDeg = motionAbsd(angDeg);
    double tArcLen = Pi * 2 * radius * angDeg / 360;
    m_linVel= tArcLen / getTimeNeeded();
    return m_linVel;
}

void CActionArc::calcInnerStrLen(double angDeg, double radius)
{
    m_innerStrLen = motionAbsd(radius * sin(deg2Rad(angDeg) / 2) * 2);
}

double CActionArc::calcYaw(double angRad)
{
    return halfPi - (Pi - angRad) / 2;
}

void CActionArc::reset(void)
{
    resetBase();

    m_endPose.reset();
    m_startUp = false;
    m_innerStrLen = 0.0;
    m_linVel = 0.0;
}
