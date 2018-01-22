/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CIdOwnedStates.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Motion idle owned state(s) 
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef CIDOWNEDSTATE_H
#define CIDOWNEDSTATE_H

#include "CState.h"
#include "CActionBase.h"
#include "motionUtils.h"
#include "CMotionIdle.h"
#include "CActionArc.h"

namespace motion
{
    class CMotionIdle;
    class CIdIdle : public CState<CMotionIdle>
    {
    public:
        ~CIdIdle(void) {}
        static CIdIdle* getInstance(void);

        virtual void enter(CMotionIdle* id);
        virtual void execute(CMotionIdle* id);
        virtual void exit(CMotionIdle* id);
    protected:

    private:
        CIdIdle(void) {}
        CIdIdle(CIdIdle&);
        CIdIdle& operator=(CIdIdle&);
    };
}
#endif /** CIDOWNEDSTATE_H */
