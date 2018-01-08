#ifndef CIDOWNEDSTATE_H
#define CIDOWNEDSTATE_H

#include "CState.h"
#include "CActionBase.h"
#include "motionUtils.h"
#include "CMotionIdle.h"
#include "CActionArc.h"

namespace motion
{
    class CMotionIdle;
    /** Idle State. */
    class CIdIdle : public CState<CMotionIdle>
    {
    public:
        ~CIdIdle(void) {}
        static CIdIdle* getInstance(void);

        virtual void enter(CMotionIdle* id);
        virtual void execute(CMotionIdle* id);
        virtual void exit(CMotionIdle* id);

    protected:

    private:
        CIdIdle(void) {}
        CIdIdle(CIdIdle&);
        CIdIdle& operator=(CIdIdle&);
    };
}
#endif /** CIDOWNEDSTATE_H */
