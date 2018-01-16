#ifndef CMOTION_H_
#define CMOTION_H_

#include "motionEnums.h"
#include "CActionBase.h"
#include "CStateMachine.h"
#include "CMotionOwnedStates.h"

#define mtn motion::CMotion::getInstance()

namespace motion
{
    class CMotion : public CActionBase
    {
    public:
        static CMotion* getInstance(void);
        virtual ~CMotion(void) { delete m_pStateMachine; }

        void update(void);
        void reset(void);

        CStateMachine<CMotion>* getFSM(void) const { return m_pStateMachine; }

    protected:

    private:
        CMotion(void);
        CMotion(const CMotion&);
        CMotion& operator=(const CMotion&);

        CStateMachine<CMotion>* m_pStateMachine;  /**< State machine of motion.*/
    };
}
#endif  /** CMOTION_H_ */
