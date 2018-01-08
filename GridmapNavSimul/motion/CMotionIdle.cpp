#include "CMotionIdle.h"

using namespace motion;

CMotionIdle::CMotionIdle(void)
{
    ///m_pStateMachine = new CStateMachine<CMotionIdle>(this);
    ///m_pStateMachine->setCurrentState(CIdIdle::getInstance());
}

CMotionIdle* CMotionIdle::getInstance(void)
{
    static CMotionIdle s_instance;
    return &s_instance;
}

void CMotionIdle::enter(CMotion* mt)
{
    setMotion(MOTION_IDLE);
    printf("[ljh] Entering motion IDLE. \n");
}

void CMotionIdle::execute(CMotion* mt)
{

}

void CMotionIdle::exit(CMotion* mt)
{
    printf("[ljh] Exiting motion IDLE. \n");
}

