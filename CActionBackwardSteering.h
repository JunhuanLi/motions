#ifndef CACTIONBACKWARDSTEERING_H_
#define CACTIONBACKWARDSTEERING_H_

#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/utils/CTicTac.h>
#include "TMotionPose2D.h"
#include "motionEnums.h"

namespace motion
{
    class CActionBackwardSteering
    {
    public:
        static CActionBackwardSteering* getInstance();
        virtual ~CActionBackwardSteering();

        void bkwdInit();
        ESteeringStateType moveBackward(
            mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
            double v, double distance);
        ESteeringStateType getState() { return m_state; }
        void setState(ESteeringStateType val) { m_state = val; }
        double getLinearVelocity() { return m_linearVelocity; }
        void setLinearVelocity(double val) { m_linearVelocity = val; }
        double getAngularVelocity() { return m_angularVelocity; }
        void setAngularVelocity(double val) { m_angularVelocity = val; }

        //!tmp functions only for testing
        double getTimeNeeded() { return m_timeNeeded; }
    protected:

    private:
        CActionBackwardSteering();
        CActionBackwardSteering(const CActionBackwardSteering&);
        CActionBackwardSteering& operator=(const CActionBackwardSteering&);

        void storeStartPose(mrpt::kinematics::CVehicleSimul_DiffDriven* robot);
        void calculateEndPose(mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
                              double distance);
        void calculateTimeNeeded(double v, double distance);
        double calculateRemainingDistance(mrpt::kinematics::CVehicleSimul_DiffDriven* robot);

        ESteeringStateType m_state;
        double m_linearVelocity;
        double m_angularVelocity;
        bool m_startPoseStored;
        TMotionPose2D m_startPose;
        TMotionPose2D m_endPose;
        double m_timeNeeded;

        //!tmp variable only for testing

    };

}
#endif // FORWARDSTEERING_H_
