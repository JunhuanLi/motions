#ifndef CMOTIONWALLFOLLOWING_H_
#define CMOTIONWALLFOLLOWING_H_

#include "motionEnums.h"
#include "CActionBase.h"
#include "CState.h"
#include "CWFOwnedStates.h"
#include "CStateMachine.h"

#define wallFol motion::CMotionWallFollowing::getInstance()

namespace motion
{
    class CMotion;
    /** Wall following mode.
     * need the following linear velocity and the desired follow distance as input.
     */
    class CMotionWallFollowing : public CActionBase, public CState<CMotion>
    {
    public:
        static CMotionWallFollowing* getInstance(void);
        virtual ~CMotionWallFollowing(void) { delete m_pStateMachine; }

        virtual void enter(CMotion* mt);
        virtual void execute(CMotion* mt);
        virtual void exit(CMotion* mt);

        void reset(void);
        CStateMachine<CMotionWallFollowing>* getFSM(void) const { return m_pStateMachine; }

        /** Tune the velocity according to the distance from robot to wall.
         * @return The scale that velocity should be Tuned. It's a ratio between 0 and 1.
         */
        double tuneVelocity(void);
        /** Calculate phi which is equal to current_phi + wallFolResumeANDAngle.*/
        void calcEndPhi(double);
        /** Calculate remaining angle, when this angle approached 0, it means the angle
         * condition of \sa angNDistReached is satisfied.
         * @return Remaining angle.
         * @see angNDistReached
         */
        double calcRemAngle(void);
        /** The resume distance satisfied or not.
         * @return true if ahead distance is larger than \sa wallFolResumeDist.
         * @see angNDistReached
         */
        bool distReached(void);
        /** The resume angle and distance satisfied or not.
         * @return true if angle condition is satisfied and side distance is larger than \sa obsStopDist.
         * @see distReached
         */
        bool angNDistReached(void);

    protected:

    private:
        CMotionWallFollowing(void);
        CMotionWallFollowing(const CMotionWallFollowing&);
        CMotionWallFollowing& operator=(const CMotionWallFollowing&);

        CStateMachine<CMotionWallFollowing>* m_pStateMachine;

        EAngNDistReachedState m_andState;
        double m_endPhi;
    };
}
#endif  /** CMOTIONWALLFOLLOWING_H_ */
