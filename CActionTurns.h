#ifndef CACTIONTURNS_H_
#define CACTIONTURNS_H_

#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/utils/CTicTac.h>
#include "TMotionPose2D.h"
#include "motionEnums.h"
#include "motionUtils.h"
#include "CActionBase.h"

namespace motion
{
    class CActionTurns : public CActionBase
    {
    public:
        static CActionTurns* getInstance(void);
        virtual ~CActionTurns(void);

        ESteeringStateType rotateInPlace(
            mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
            double v, double w, double theta);

        double getAngle(void) { return m_angle; }
        void setAngle(double val) { m_angle = val; }

    protected:

    private:
        CActionTurns(void);
        CActionTurns(const CActionTurns&);
        CActionTurns& operator=(const CActionTurns&);

        void calculateEndPose(mrpt::kinematics::CVehicleSimul_DiffDriven* robot,
                              double v, double theta);
        double calculateRemainingAngle(mrpt::kinematics::CVehicleSimul_DiffDriven* robot);

        double m_angle;
        TMotionPose2D m_endPose;

        //!tmp variable only for testing
        double m_remAngle;
    };

}
#endif // CACTIONTURNS_H_
