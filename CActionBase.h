#ifndef CACTIONBASE_H
#define CACTIONBASE_H

#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include "TMotionPose2D.h"
#include "motionUtils.h"

namespace motion
{
    class CActionBase
    {
    public:
        CActionBase(void);
        virtual ~CActionBase(void);

        ESteeringStateType getState(void) { return m_state; }
        void setState(ESteeringStateType val) { m_state = val; }
        double getLinearVelocity(void) { return m_linearVelocity; }
        void setLinearVelocity(double val) { m_linearVelocity = val; }
        double getAngularVelocity(void) { return m_angularVelocity; }
        void setAngularVelocity(double val) { m_angularVelocity = val; }
        bool getStartPoseStoredFlag(void) { return m_startPoseStored; }
        TMotionPose2D getStartPose(){ return m_startPose; }

        double getTimeNeeded(void) { return m_timeNeeded; }

        void storeStartPose(mrpt::kinematics::CVehicleSimul_DiffDriven* robot);
        void calculateTimeNeeded(double velocity, double displacement);

    protected:

    private:
        ESteeringStateType m_state;
        double m_linearVelocity;
        double m_angularVelocity;
        bool m_startPoseStored;
        TMotionPose2D m_startPose;

        double m_timeNeeded;
    };
}

#endif // CACTIONBASE_H
