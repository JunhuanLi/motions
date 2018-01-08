#ifndef CPTOWNEDSTATES_H_
#define CPTOWNEDSTATES_H_

#include "CState.h"
#include "CActionBase.h"
#include "motionUtils.h"
#include "CMotion.h"
#include "CMotionPointTracker.h"
#include "CActionArc.h"

namespace motion
{
    class CMotionPointTracker;

    /** Point Tracking idle. */
    class CPTIdle : public CState<CMotionPointTracker>
    {
    public:
        ~CPTIdle(void) {}
        static CPTIdle* getInstance(void);

        virtual void enter(CMotionPointTracker* pt);
        virtual void execute(CMotionPointTracker* pt);
        virtual void exit(CMotionPointTracker* pt);

    protected:

    private:
        CPTIdle(void) {}
        CPTIdle(CPTIdle&);
        CPTIdle& operator=(CPTIdle&);
    };

    /** Point Tracking State. */
    class CPTTracking : public CState<CMotionPointTracker>
    {
    public:
        ~CPTTracking(void) {}
        static CPTTracking* getInstance(void);

        virtual void enter(CMotionPointTracker* pt);
        virtual void execute(CMotionPointTracker* pt);
        virtual void exit(CMotionPointTracker* pt);

    protected:

    private:
        CPTTracking(void) {}
        CPTTracking(CPTTracking&);
        CPTTracking& operator=(CPTTracking&);
    };
}
#endif /** CPTOWNEDSTATES_H_ */
