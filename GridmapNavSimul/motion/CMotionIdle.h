/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CMotionIdle.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion idle
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
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
   /** 
    * Motion idle state \n
    *
    * example: \n
    * // To change to this state from other state \n
    *
    * #include CMotion.h \n
    *
    * mtn->setMotion(MOTION_IDLE); \n
    *
    */
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

        ///CStateMachine<CMotionIdle>* m_pStateMachine;  /**< State machine of idle.*/

    };
}
#endif /** CMOTIONIDLE_H_ */
