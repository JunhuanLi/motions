/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CStateMachine.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Finite state machine
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef CSTATEMACHINE_H
#define CSTATEMACHINE_H

#include <cassert>
#include <string>

#include "CState.h"


namespace motion
{
	template <class state_type>
	class CStateMachine
	{
	public:
	  CStateMachine(state_type* owner):m_pOwner(owner),
	                                   m_pCurrentState(NULL),
	                                   m_pPreviousState(NULL),
	                                   m_pGlobalState(NULL)
	  {}

	  virtual ~CStateMachine() {}

	  /** Use these methods to initialize the FSM. */
	  void setCurrentState(CState<state_type>* s) { m_pCurrentState = s; }
	  void setGlobalState(CState<state_type>* s) { m_pGlobalState = s; }
	  void setPreviousState(CState<state_type>* s) { m_pPreviousState = s; }

	  /** Call this to update the FSM. */
	  void  update() const
	  {
	    /** If a global state exists, call its execute method, else do nothing. */
	    ///if(m_pGlobalState)   m_pGlobalState->execute(m_pOwner);

	    /** Same for the current state. */
	    if (m_pCurrentState) m_pCurrentState->execute(m_pOwner);
	  }

	  /** Change to a new state. */
	  void  changeState(CState<state_type>* pNewState)
	  {
	    assert(pNewState &&
	           "<CStateMachine::changeState>: trying to change to NULL state.\n");

	    /** Keep a record of the previous state. */
	    m_pPreviousState = m_pCurrentState;

	    /** Call the exit method of the existing state. */
	    m_pCurrentState->exit(m_pOwner);

	    /** Change state to the new state. */
	    m_pCurrentState = pNewState;

	    /** Call the entry method of the new state. */
	    m_pCurrentState->enter(m_pOwner);
	  }

	  /** Change state back to the previous state. */
	  void  revertToPreviousState()
	  {
	    changeState(m_pPreviousState);
	  }

	  /** Returns true if the current state's type is equal to the type of the
	   * class passed as a parameter.
	   */
///	  bool  isInState(const CState<state_type>& st) const
///	  {
///	    return typeid(*m_pCurrentState) == typeid(st);
///	  }

	  CState<state_type>*  getCurrentState(void) const{ return m_pCurrentState; }
	  CState<state_type>*  getGlobalState(void) const{ return m_pGlobalState; }
	  CState<state_type>*  getPreviousState(void) const{ return m_pPreviousState; }

	private:
	  state_type*          m_pOwner;  /**< A pointer to the agent that owns this instance. */
	  CState<state_type>*   m_pCurrentState;  /**< A record of current state. */
	  CState<state_type>*   m_pPreviousState;  /**< A record of the last state the agent was in. */
	  CState<state_type>*   m_pGlobalState;  /**< This is called every time the FSM is updated. */
	};
}

#endif /** CSTATEMACHINE_H */
