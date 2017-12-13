#include "CActionBase.h"
using namespace motion;

CActionBase::CActionBase(void)
{
    //ctor
}

CActionBase::~CActionBase(void)
{
    //dtor
}

void CActionBase::storeStartPose(
    mrpt::kinematics::CVehicleSimul_DiffDriven* robot)
{
    m_startPose.x = robot->getCurrentGTPose().x;
    m_startPose.y = robot->getCurrentGTPose().y;
    m_startPose.phi = robot->getCurrentGTPose().phi;
    m_startPoseStored = true;  /** set the pose-stored flag */
}

void CActionBase::calculateTimeNeeded(double velocity, double displacement)
{
    m_timeNeeded = displacement / abs(velocity);
}
