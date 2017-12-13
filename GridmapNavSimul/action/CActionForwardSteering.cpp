#include "CActionForwardSteering.h"
using namespace motion;

CActionForwardSteering::CActionForwardSteering(void)
{
    //ctor
}

CActionForwardSteering::~CActionForwardSteering(void)
{
    //dtor
}

CActionForwardSteering* CActionForwardSteering::getInstance(void)
{
    static CActionForwardSteering s_instance;
    return &s_instance;
}

ESteeringStateType CActionForwardSteering::moveForward(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
    double v,
    double distance)
{
    static mrpt::utils::CTicTac	s_tictac;

    if(!getStartPoseStoredFlag())
    {
        storeStartPose(robot);
        calculateEndPose(robot, distance);
        calculateTimeNeeded(v, distance);

        s_tictac.Tic();  /** start the timer */
    }

    if(s_tictac.Tac() > getTimeNeeded() + 1)
    {
        //throw
        setLinearVelocity(0.0);
        setState(FAILED);
        return getState();
    }

    if(calculateRemainingDistance(robot) <= 0.05)
    {
        setLinearVelocity(0.0);
        //!m_startPoseStored = false;
        setState(SUCCESS);
    }
    else
    {
        setLinearVelocity(v);
        setState(EXECUTING);
    }

    return getState();
}

void CActionForwardSteering::calculateEndPose(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
    double distance)
{
    m_endPose.x = getStartPose().x + distance*cos(getStartPose().phi);
    m_endPose.y = getStartPose().y + distance*sin(getStartPose().phi);
    m_endPose.phi = robot->getCurrentGTPose().phi;
}

double CActionForwardSteering::calculateRemainingDistance(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot)
{
    double curDistX = (robot->getCurrentGTPose().x - m_endPose.x)
                      * (robot->getCurrentGTPose().x - m_endPose.x);
    double curDistY = (robot->getCurrentGTPose().y - m_endPose.y)
                      * (robot->getCurrentGTPose().y - m_endPose.y);
    return sqrt(curDistX + curDistY);
}


