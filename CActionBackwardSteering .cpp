#include "CActionBackwardSteering.h"
using namespace motion;

CActionBackwardSteering::CActionBackwardSteering()
{
    //ctor
}

CActionBackwardSteering::~CActionBackwardSteering()
{
    //dtor
}

CActionBackwardSteering* CActionBackwardSteering::getInstance()
{
    static CActionBackwardSteering s_instance;
    return &s_instance;
}

ESteeringStateType CActionBackwardSteering::moveBackward(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
    double v,
    double distance)
{
    static mrpt::utils::CTicTac	s_tictac;

    if(!m_startPoseStored)
    {
        storeStartPose(robot);
        calculateEndPose(robot, distance);
        calculateTimeNeeded(v, distance);

        s_tictac.Tic();  //!start the timer
        m_startPoseStored = true;  //!set the pose-stored flag
    }

    if(s_tictac.Tac() > m_timeNeeded + 1)
    {
        //throw
        m_linearVelocity = 0.0;
        m_state = FAILED;
        return m_state;
    }

    if(calculateRemainingDistance(robot) <= 0.05)
    {
        m_linearVelocity = 0.0;
        //!m_startPoseStored = false;
        m_state = SUCCESS;
    }
    else
    {
        m_linearVelocity = -v;
        m_state = EXECUTING;
    }

    return m_state;
}

void CActionBackwardSteering::storeStartPose(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot)
{
    m_startPose.x = robot->getCurrentGTPose().x;
    m_startPose.y = robot->getCurrentGTPose().y;
    m_startPose.phi = robot->getCurrentGTPose().phi;
}

void CActionBackwardSteering::calculateEndPose(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
    double distance)
{
    m_endPose.x = m_startPose.x + distance*cos(m_startPose.phi);
    m_endPose.y = m_startPose.y + distance*sin(m_startPose.phi);
    m_endPose.phi = robot->getCurrentGTPose().phi;
}

double CActionBackwardSteering::calculateRemainingDistance(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot)
{
    double curDistX = (robot->getCurrentGTPose().x - m_endPose.x)
                      * (robot->getCurrentGTPose().x - m_endPose.x);
    double curDistY = (robot->getCurrentGTPose().y - m_endPose.y)
                      * (robot->getCurrentGTPose().y - m_endPose.y);
    return sqrt(curDistX + curDistY);
}

void CActionBackwardSteering::calculateTimeNeeded(double v, double distance)
{
    m_timeNeeded = distance / abs(v);
}
