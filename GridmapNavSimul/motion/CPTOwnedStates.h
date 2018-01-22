/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CPTOwnedStates.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion point tracker owned state(s)
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef CPTOWNEDSTATES_H_
#define CPTOWNEDSTATES_H_

#include "CState.h"
#include "CActionBase.h"
#include "motionUtils.h"
#include "CMotion.h"
#include "CMotionPointTracker.h"
#include "CActionArc.h"
#include "TMotionPIDCon.h"

namespace motion
{
    class CMotionPointTracker;

    /** Point Tracking idle state. 
     * It will change to this state automatically when a point tracking task finished.
     */
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
