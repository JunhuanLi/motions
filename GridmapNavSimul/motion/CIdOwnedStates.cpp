#include "CIdOwnedStates.h"

using namespace motion;

/** Idle Methods. */
CIdIdle* CIdIdle::getInstance(void)
{
    static CIdIdle s_instance;
    return &s_instance;
}

void CIdIdle::enter(CMotionIdle* id)
{

    printf("[ljh] Entering CIdIdle.\n");
}

void CIdIdle::execute(CMotionIdle* id)
{
    /** Do nothing. */
    printf("[ljh] Executing CIdIdle.\n");
    sleep(3);
}

void CIdIdle::exit(CMotionIdle* id)
{
    printf("[ljh] Exiting CIdIdle.\n");
}
