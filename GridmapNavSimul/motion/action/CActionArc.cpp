/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CActionArc.cpp
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Arc action
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
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

    /// Store current information and calculate information needed.
    if(!getPoseStored())
    {
        reset();
        setAction(ARC);
        m_commandAngDeg = angDeg;
        startUp(angDeg);
        storeStartPose();
        calcTimeNeeded(w, deg2Rad(angDeg));
        calcInnerStrLen(angDeg, radius);
        m_linVel = calcLinVel(angDeg, radius);
        calcEndPose(angDeg);

        s_tictac.Tic();  /// Start the timer
    }

    /// If start up is enabled, update if the vehicle get started everytime.
    if(getStartUp())
        updateStartUp();

    /// Timeout check.
    if(s_tictac.Tac() > getTimeNeeded() + 1)
    {
        ///throw exceptions
        ///assert(!"Time out when making Arc move.");
        printf("[ljh] Arc timeOut.\n");
        reset();
        setActionState(ARC_FAILED);
        return;
    }

    /// Finish condition check.
    if(calcRem() <= slightTuneAngDeg && !m_slightTuneOn)
    {
        m_slightTuneOn = true;
        printf("[ljh] slight tune on.\n");
    }

    if(!getStartUp() && m_slightTuneFinished)
    {
        printf("[ljh] Direction reached.\n");
        if(poseVerified())
        {
            printf("[ljh] Pose verify passed.\n");
            reset();
            setActionState(ARC_FINISHED);
            setAction(STOP);
        }
        else
        {
            printf("[ljh] Pose verify not passed.\n");
            ///throw exceptions
            ///assert(!"Direction reached but can't reach the appropriate location.");
            reset();
            setActionState(ARC_FAILED);
        }
    }
    else
    {
        if(!m_slightTuneOn)
        {
            setVelocity(motionAbsd(m_linVel), motionAbsd(w) * getRotSide(angDeg));
        }
        else
        {
            if(m_angRem <= -angAccuracy)
            {
                setVelocity(0, -motionAbsd(w));
            }
            else if(m_angRem >= angAccuracy)
            {
                setVelocity(0, motionAbsd(w));
            }
            else
            {
                m_slightTuneFinished = true;
            }

        }
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
    m_myCurPhi = getCurPose().phi + twoPi * getJumpDir() * getJumpCount();
    m_phiRotated = m_myCurPhi - getStartPose().phi;
    ///printf("[ljh] curphi: %.3f, startPhi: %.3f\n", m_myCurPhi, getStartPose().phi);
    m_angRem = m_commandAngDeg - rad2Deg(m_phiRotated);
    ///printf("[ljh] comang: %.3f, rotdang: %.3f\n", m_commandAngDeg, rad2Deg(m_phiRotated));
    ///printf("[ljh] remain angle: %.3f\n", m_angRem);
    return motionAbsd(m_angRem);
    ///return motionAbsd(rad2Deg(m_endPose.phi - getCurPose().phi));
 }

bool CActionArc::poseVerified(void)
{
    double tDist = sqrt((getCurPose().x - m_endPose.x)
                           * (getCurPose().x - m_endPose.x)
                           + (getCurPose().y - m_endPose.y)
                           * (getCurPose().y - m_endPose.y));

    if(tDist <= locAccuracy) return true;
    else return false;
}

void CActionArc::startUp(double angDeg)
{
    if(motionAbsd(rad2Deg(twoPi) - motionAbsd(angDeg)) <= startUpAngDeg)
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
    if(rad2Deg(m_phiRotated) > startUpAngDeg)
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
    m_slightTuneOn = false;
    m_startUp = false;
    m_innerStrLen = 0.0;
    m_linVel = 0.0;
}
