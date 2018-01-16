#ifndef CMOTIONPOINTTRACKER_H_
#define CMOTIONPOINTTRACKER_H_

#include "CActionBase.h"
#include "TMotionPose2D.h"
#include "CStateMachine.h"
#include "CState.h"
#include "motionEnums.h"
#include "CPTOwnedStates.h"

#define mpt motion::CMotionPointTracker::getInstance()

namespace motion
{
    class CMotion;

    class CMotionPointTracker : public CActionBase, public CState<CMotion>
    {
    public:
        static CMotionPointTracker* getInstance(void);
        virtual ~CMotionPointTracker(void) { delete m_pStateMachine; }

        virtual void enter(CMotion* mt);
        virtual void execute(CMotion* mt);
        virtual void exit(CMotion* mt);

        void reset(void);
        int arcSide(void) { return m_arcSide; }
        double getDist(void) const { return m_dist; }
        ELinearPTState getlptState(void) const { return m_lptState; }
        void setlptState(ELinearPTState s) { m_lptState = s; }
        CStateMachine<CMotionPointTracker>* getFSM(void) const { return m_pStateMachine; }

        /** Calculate \sa m_angle. */
        void calcPhi(void);
        /** Calculate \sa m_dist. */
        void calcDist(void);
        /** Make the robot turn to the next point. */
        void turn2TargPoint(void);
        /** Make the robot translate to the next point. */
        void trackPoint(void);
        /** Arc point tracking method. */
        void ArcPT(void);
        /** Linear point tracking method. */
        void LinearPT(void);

    protected:

    private:
        CMotionPointTracker(void);
        CMotionPointTracker(CMotionPointTracker&);
        CMotionPointTracker& operator=(CMotionPointTracker&);

        CStateMachine<CMotionPointTracker>* m_pStateMachine;  /**< State machine of point tracking. */

        ELinearPTState m_lptState;  /**< Linear point tracking state.*/
        int m_arcSide;  /**< The arc side.*/
        double m_angle;  /**< This is the angle that the robot need to rotate in order to face the next point.*/
        double m_dist;  /**< Robot distance from the next point.*/
    };
}

#endif /** CMOTIONPOINTTRACKER_H_ */
