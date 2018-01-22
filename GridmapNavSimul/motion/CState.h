/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CState.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief State interface
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef CSTATE_H_
#define CSTATE_H_

namespace motion
{
    template <class state_type>
    class CState
    {
    public:

      virtual ~CState(){}

      /** This will execute when the state is entered. */
      virtual void enter(state_type*)=0;

      /** This is the states normal update function. */
      virtual void execute(state_type*)=0;

      /** This will execute when the state is exited. */
      virtual void exit(state_type*)=0;
    };
}

#endif /** CSTATE_H_ */
