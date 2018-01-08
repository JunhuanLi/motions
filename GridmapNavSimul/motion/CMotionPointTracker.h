#ifndef CMOTIONPOINTTRACKER_H_
#define CMOTIONPOINTTRACKER_H_

#include "CActionBase.h"
#include "TMotionPose2D.h"
#include "CStateMachine.h"
#include "CState.h"
#include "motionEnums.h"
#include "CPTOwnedStates.h"

#define pointTra motion::CMotionPointTracker::getInstance()

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

        ///void setPoint(double, double, EPTMode mode=LPT);
        void reset(void);
        int arcSide(void) { return m_arcSide; }
        double getDist(void) const { return m_dist; }
        ELinearPTState getlptState(void) const { return m_lptState; }
        void setlptState(ELinearPTState s) { m_lptState = s; }
        CStateMachine<CMotionPointTracker>* getFSM(void) const { return m_pStateMachine; }

        void storeStartPose(void);
        void calcPhi(void);
        void calcDist(void);
        void turn2TargPoint(void);
        void trackPoint(void);

        void ArcPT(void);
        void LinearPT(void);

    protected:

    private:
        CMotionPointTracker(void);
        CMotionPointTracker(CMotionPointTracker&);
        CMotionPointTracker& operator=(CMotionPointTracker&);

        CStateMachine<CMotionPointTracker>* m_pStateMachine;

        ELinearPTState m_lptState;
        int m_arcSide;
        double m_angle;
        double m_dist;
    };
}

#endif /** CMOTIONPOINTTRACKER_H_ */
