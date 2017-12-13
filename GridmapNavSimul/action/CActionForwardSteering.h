#ifndef CACTIONFORWARDSTEERING_H_
#define CACTIONFORWARDSTEERING_H_

#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/utils/CTicTac.h>
#include "TMotionPose2D.h"
#include "motionEnums.h"
#include "CActionBase.h"

namespace motion
{
    class CActionForwardSteering : public CActionBase
    {
    public:
        static CActionForwardSteering* getInstance(void);
        virtual ~CActionForwardSteering(void);

        ESteeringStateType moveForward(
            mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
            double v, double distance);

    protected:

    private:
        CActionForwardSteering(void);
        CActionForwardSteering(const CActionForwardSteering&);
        CActionForwardSteering& operator=(const CActionForwardSteering&);

        TMotionPose2D m_endPose;

        void calculateEndPose(mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
                              double distance);

        double calculateRemainingDistance(mrpt::kinematics::CVehicleSimul_DiffDriven* robot);
    };

}
#endif // CACTIONFORWARDSTEERING_H_
