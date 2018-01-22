/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CWFOwnedStates.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion wall following owned state(s)
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
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

    /** Obstacle Avoidance State. 
     * When the ahead distance is small enough, sub-motion will change to this state. 
     */
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

    /** Ridge Tracking State. 
     * When the side distance have big jump, sub-motion will change to this state. 
     */
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


