/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CMotionWallFollowing.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion wall following
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef CMOTIONWALLFOLLOWING_H_
#define CMOTIONWALLFOLLOWING_H_

#include "motionEnums.h"
#include "CActionBase.h"
#include "CState.h"
#include "CWFOwnedStates.h"
#include "CStateMachine.h"
#include "TMotionPIDCon.h"

#define wallFol motion::CMotionWallFollowing::getInstance()

namespace motion
{
    class CMotion;
    /** Wall following mode.
     * Need the following linear velocity and the desired follow distance as input.
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
        TMotionPIDCon getWFController(void) const { return wfController; }

        /** Tune the velocity according to the distance from robot to wall.
         * @return The scale that velocity should be Tuned. It's a ratio between 0 and 1.
         */
        double tuneVelocity(void);
        /** Calculate remaining angle, when this angle approached 0, it means the angle
         * condition of angNDistReached is satisfied.
         * @return Remaining angle.
         * @see angNDistReached
         */
        double calcRemAngle(void);
        /** The resume distance satisfied or not.
         * @return true if ahead distance is larger than wallFolResumeDist.
         * @see angNDistReached 
         * @see wallFolResumeDist
         */
        bool distReached(void);
        /** The resume angle and distance satisfied or not.
         * @return true if angle condition is satisfied and side distance is larger than obsStopDist.
         * @see distReached 
         * @see obsStopDist
         */
        bool angNDistReached(void);

    protected:

    private:
        CMotionWallFollowing(void);
        CMotionWallFollowing(const CMotionWallFollowing&);
        CMotionWallFollowing& operator=(const CMotionWallFollowing&);

        CStateMachine<CMotionWallFollowing>* m_pStateMachine;  /**< State machine of wall followign. */

        TMotionPIDCon wfController;  /**< Wall following controller.*/
        EAngNDistReachedState m_andState;  /**< Angle and distance state. */
    };
}
#endif  /** CMOTIONWALLFOLLOWING_H_ */
