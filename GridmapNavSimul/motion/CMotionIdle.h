#ifndef CMOTIONIDLE_H_
#define CMOTIONIDLE_H_

#include "CActionBase.h"
#include "CIdOwnedStates.h"
#include "CState.h"
#include "CStateMachine.h"

#define motIdle motion::CMotionIdle::getInstance()

namespace motion
{
    class CMotion;
    /** Idle mode. */
    class CMotionIdle : public CActionBase, public CState<CMotion>
    {
    public:
        static CMotionIdle* getInstance(void);
        virtual ~CMotionIdle(void) { /**delete m_pStateMachine;*/  }

        virtual void enter(CMotion* mt);
        virtual void execute(CMotion* mt);
        virtual void exit(CMotion* mt);

        ///CStateMachine<CMotionIdle>* getFSM(void) const { return m_pStateMachine; }

    protected:

    private:
        CMotionIdle(void);
        CMotionIdle(const CMotionIdle&);
        CMotionIdle& operator=(const CMotionIdle&);

        ///CStateMachine<CMotionIdle>* m_pStateMachine;

    };
}
#endif // CMOTIONIDLE_H_
