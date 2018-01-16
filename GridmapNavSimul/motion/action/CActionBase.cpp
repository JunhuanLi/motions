#include "CActionBase.h"
#include "../CMotion.h"
#include "../CMotionOwnedStates.h"
#include "../CPTOwnedStates.h"

using namespace motion;

TMotion CActionBase::sm_motion;

CActionBase::CActionBase(void)
{
    //ctor
}

CActionBase::~CActionBase(void)
{
    //dtor
}

void CActionBase::storeStartPose(void)
{
    sm_motion.startPose = sm_motion.curPose;
    sm_motion.poseStored = true;  /// Set the pose-stored flag.
}

void CActionBase::calcTimeNeeded(double velocity, double displacement)
{
    sm_motion.timeNeeded = motionAbsd(displacement / velocity);
}

void CActionBase::captureJump(void)
{
    double jumpAngDeg = rad2Deg(getCurPose().phi - sm_motion.prePhi);
    ///printf("**jump is: %.3f\n", jumpAngDeg);
    if(jumpAngDeg > jumpThreshold)
    {
        if(!sm_motion.positiveSet)
        {
            sm_motion.positive = ROTATECW;
            sm_motion.positiveSet = true;
            printf("**CW positive set!\n");
        }

        if(sm_motion.positive == ROTATECW)
            sm_motion.jumpCount++;
        else
            sm_motion.jumpCount--;
        printf("**CW jump count: %d\n", sm_motion.jumpCount);

        sm_motion.jumpDirection = ROTATECW;
    }
    else if(jumpAngDeg < -jumpThreshold)
    {
        if(!sm_motion.positiveSet)
        {
            sm_motion.positive = ROTATECCW;
            sm_motion.positiveSet = true;
            printf("**CCW positive set!\n");
        }

        if(sm_motion.positive == ROTATECCW)
            sm_motion.jumpCount++;
        else
            sm_motion.jumpCount--;

        printf("**CCW jump count: %d\n", sm_motion.jumpCount);
        sm_motion.jumpDirection = ROTATECCW;
    }
    else
    {
       /// printf("no jumps.\n");
    }

    sm_motion.prePhi = getCurPose().phi;
}

void CActionBase::updateCurPose(mrpt::kinematics::CVehicleSimul_DiffDriven* robot)
{
    sm_motion.curPose.x = robot->getCurrentGTPose().x;
    sm_motion.curPose.y = robot->getCurrentGTPose().y;
    sm_motion.curPose.phi = robot->getCurrentGTPose().phi;

    captureJump();
}

void CActionBase::updateDist(
    mrpt::obs::CObservation2DRangeScan* scan, size_t winLength)
{
    /** Calculate side distance.*/
    double tDist = 0;

    for(size_t i = infraredRayAngle; i < infraredRayAngle+winLength; i++)
    {
        tDist += scan->getScanRange(i);
    }
    sm_motion.sideDist = tDist/winLength*cos(deg2Rad(infraredRayAngle));
    ///printf("sideDist: %.3f\n", sm_motion.sideDist);

    /** Calculate ahead distance.*/
    tDist = 0;
    size_t tMid = (scan->getScanSize()-1)/2;
    size_t tStart = tMid - winLength/2;
    size_t tEnd = tMid + winLength/2 + 1;
    for(size_t i = tStart; i < tEnd; i++)
    {
        tDist += scan->getScanRange(i);
    }

    sm_motion.aheadDist = tDist/winLength;
}

TMotionPose2D CActionBase::frameRot(double x, double y, double phiRad)
{
    TMotionPose2D poseRes;

    /** Rotation matrix:
     *        |  cos(0) sin(0) 0 |
     *    R = | -sin(0) cos(0) 0 |
     *        |     0       0  1 |
     */
    poseRes.x =  x * cos(phiRad) + y * sin(phiRad);
    poseRes.y = -x * sin(phiRad) + y * cos(phiRad);
    poseRes.phi = phiRad;
    return poseRes;
}

void CActionBase::setMotionParams(TMotionParams param)
{
    sm_motion.motParams = param;
    switch(getMotion())
    {
        case MOTION_IDLE:
            ///NOTHING
            break;

        case WALL_FOLLOWING:
            wallFol->getFSM()->changeState(CWFWallFol::getInstance());
            break;

        case POINT_TRACKING:
            mpt->getFSM()->changeState(CPTTracking::getInstance());
            break;

        default:
            assert(!"Invalid motion, can't set params. Please set correct motion first.\n");
            break;
    }
}

void CActionBase::setMotion(EMotion mot)
{
    sm_motion.mot = mot;
    switch(mot)
    {
        case MOTION_IDLE:
            mtn->getFSM()->changeState(CMotionIdle::getInstance());
            break;

        case WALL_FOLLOWING:
            mtn->getFSM()->changeState(CMotionWallFollowing::getInstance());
            break;

        case POINT_TRACKING:
            mtn->getFSM()->changeState(CMotionPointTracker::getInstance());
            break;

        default:
            assert(!"Invalid Motion.\n");
            break;
    }
}

void CActionBase::setVelocity(double v, double w)
{
    sm_motion.linearVelocity = v;
    sm_motion.angularVelocity = w;
}

void CActionBase::resetBase()
{
    sm_motion.linearVelocity = 0.0;
    sm_motion.angularVelocity = 0.0;
    sm_motion.timeNeeded = 0.0;
    sm_motion.poseStored = false;
    sm_motion.jumpCount = 0;
    sm_motion.positiveSet = false;
}
