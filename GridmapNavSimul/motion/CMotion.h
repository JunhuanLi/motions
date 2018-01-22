/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CMotion.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion (owner of any motion)
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef CMOTION_H_
#define CMOTION_H_

#include "motionEnums.h"
#include "CActionBase.h"
#include "CStateMachine.h"
#include "CMotionOwnedStates.h"

#define mtn motion::CMotion::getInstance()

namespace motion
{   
   /** 
    * Motion \n
    *
    * example: \n
    *
    * //to update motion
    * mtn->update(); \n
    *
    */
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
