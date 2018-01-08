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
