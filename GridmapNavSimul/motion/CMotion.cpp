#include "CMotion.h"

using namespace motion;

CMotion::CMotion(void)
{
    m_pStateMachine = new CStateMachine<CMotion>(this);
    m_pStateMachine->setCurrentState(CMotionIdle::getInstance());
}

CMotion* CMotion::getInstance(void)
{
    static CMotion s_instance;
    return &s_instance;
}

void CMotion::update(void)
{
    /**Check bump and lift later here.*/

    m_pStateMachine->update();
}

void CMotion::reset()
{

}
