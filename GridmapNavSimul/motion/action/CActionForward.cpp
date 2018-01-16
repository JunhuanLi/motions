#include "CActionForward.h"
using namespace motion;

CActionForward* CActionForward::getInstance(void)
{
    static CActionForward s_instance;
    return &s_instance;
}

void CActionForward::forward(double v, double distance)
{
    static mrpt::utils::CTicTac	s_tictac;

    /** Store current information and calculate information needed*/
    if(!getPoseStored())
    {
        reset();
        setAction(FORWARD);
        storeStartPose();
        calcEndPose(motionAbsd(distance));
        calcTimeNeeded(v, distance);

        s_tictac.Tic();  /// start the timer
    }

    /** Timeout check.*/
    if(s_tictac.Tac() > getTimeNeeded() + 1)
    {
        //throw
        printf("[ljh] Forward timeOut.\n");
        setVelocity(0.0, 0.0);
        setPoseStored(false);
        setActionState(FORWARD_FAILED);
        return;
    }

    /** Finish condition check.*/
    if(calcRem() <= locAccuracy)
    {
        printf("[ljh] Forward finished.\n");
        setVelocity(0.0, 0.0);
        setPoseStored(false);
        setActionState(FORWARD_FINISHED);
        setAction(STOP);
    }
    else
    {
        ///todo w!=0
        setVelocity(motionAbsd(v), 0.0);
        setActionState(FORWARD_EXECUTING);
    }
}

void CActionForward::calcEndPose(double distance)
{
    TMotionPose2D absPose = frameRot(distance, 0, -getCurPose().phi);
    TMotionPose2D startPose = getStartPose();
    m_endPose = startPose + absPose;
}

double CActionForward::calcRem()
{
    double curDistX = (getCurPose().x - m_endPose.x)
                      * (getCurPose().x - m_endPose.x);
    double curDistY = (getCurPose().y - m_endPose.y)
                      * (getCurPose().y - m_endPose.y);
    return sqrt(curDistX + curDistY);
}

void CActionForward::reset()
{
    resetBase();  //!< Reset base private variables.

    m_endPose.reset();
}
