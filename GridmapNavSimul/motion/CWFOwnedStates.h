#ifndef CWFOWNEDSTATES_H_
#define CWFOWNEDSTATES_H_

#include "CState.h"
#include "CActionBase.h"
#include "motionUtils.h"
#include "CMotionWallFollowing.h"

namespace motion
{
    class CMotionWallFollowing;

    /** Wall Following State. */
    class CWFWallFol : public CState<CMotionWallFollowing>
    {
    public:
        ~CWFWallFol(void) {}
        static CWFWallFol* getInstance(void);

        virtual void enter(CMotionWallFollowing* wf);
        virtual void execute(CMotionWallFollowing* wf);
        virtual void exit(CMotionWallFollowing* wf);

    protected:

    private:
        CWFWallFol(void) {}
        CWFWallFol(CWFWallFol&);
        CWFWallFol& operator=(CWFWallFol&);
    };

    /** Obstacle Avoidance State. */
    class CWFObsAvoid : public CState<CMotionWallFollowing>
    {
    public:
        ~CWFObsAvoid(void){}
        static CWFObsAvoid* getInstance(void);

        virtual void enter(CMotionWallFollowing* wf);
        virtual void execute(CMotionWallFollowing* wf);
        virtual void exit(CMotionWallFollowing* wf);

    protected:

    private:
        CWFObsAvoid(void) {}
        CWFObsAvoid(CWFObsAvoid&);
        CWFObsAvoid& operator=(CWFObsAvoid&);
    };

    /** Ridge Tracking State. */
    class CWFRidgeTrack : public CState<CMotionWallFollowing>
    {
    public:
        ~CWFRidgeTrack(void) {}
        static CWFRidgeTrack* getInstance(void);

        virtual void enter(CMotionWallFollowing* wf);
        virtual void execute(CMotionWallFollowing* wf);
        virtual void exit(CMotionWallFollowing* wf);

    protected:

    private:
        CWFRidgeTrack(void) {}
        CWFRidgeTrack(CWFRidgeTrack&);
        CWFRidgeTrack& operator=(CWFRidgeTrack&);
    };
}
#endif /** CWFOWNEDSTATES_H_ */


