#include "CActionTurns.h"
using namespace motion;

CActionTurns::CActionTurns(void)
{
    //ctor
}

CActionTurns::~CActionTurns(void)
{
    //dtor
}

CActionTurns* CActionTurns::getInstance(void)
{
    static CActionTurns s_instance;
    return &s_instance;
}

ESteeringStateType CActionTurns::rotateInPlace(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
    double v, double w, double theta)
{
    static mrpt::utils::CTicTac	s_tictac;

    if(!getStartPoseStoredFlag())
    {
        storeStartPose(robot);
        calculateEndPose(robot, v, deg2Rad(theta));
        calculateTimeNeeded(w, deg2Rad(theta));

        s_tictac.Tic();  //!start the timer
    }

    if(s_tictac.Tac() > getTimeNeeded() + 1)
    {
        //throw
        setLinearVelocity(0.0);
        setAngularVelocity(0.0);
        setState(FAILED);
        return getState();
    }

    if(calculateRemainingAngle(robot) <= 3)
    {
        setLinearVelocity(0.0);
        setAngularVelocity(0.0);
        //!m_startPoseStored = false;
        setState(SUCCESS);
    }
    else
    {
        setLinearVelocity(v);
        setAngularVelocity(w);
        setState(EXECUTING);
    }

    return getState();
}

void CActionTurns::calculateEndPose(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
    double v, double theta)
{
    //m_endPose.x = m_startPose.x + (m_timeNeeded * v) * cos(m_startPose.phi);
    //m_endPose.y = m_startPose.y + (m_timeNeeded * v) * sin(m_startPose.phi);
    m_endPose.phi = getStartPose().phi + theta;
}

double CActionTurns::calculateRemainingAngle(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot)
{
    double remainingAngle = m_endPose.phi - (robot->getCurrentGTPose().phi - getStartPose().phi);
    m_remAngle = rad2Deg(remainingAngle);
    return rad2Deg(remainingAngle);
}
