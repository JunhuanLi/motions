#ifndef CACTIONBASE_H_
#define CACTIONBASE_H_

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include "TMotionPose2D.h"
#include "motionUtils.h"
#include "motionEnums.h"
#include "TMotion.h"
#include "TMotionParams.h"

namespace motion
{
    class CActionBase
    {
    public:
        CActionBase(void);
        virtual ~CActionBase(void);

        void resetBase(void);
        double getAheadDist(void) const { return sm_motion.aheadDist; }
        double getSideDist(void) const { return sm_motion.sideDist; }
        double getLinearVelocity(void) const { return sm_motion.linearVelocity; }
        void setVelocity(double v, double w);

        void setMotion(EMotion mot);
        void setSubMotion(ESubMotion submot) { sm_motion.subMot = submot; }
        void setSubMotionState(ESubMotionState submotstate) { sm_motion.subMotState = submotstate; }
        void setAction(EAction act) const { sm_motion.act = act; }
        void setActionState(EActionState actsta) { sm_motion.actState = actsta; }
        EMotion getMotion(void) const { return sm_motion.mot; }
        ESubMotion getSubMotion(void) const { return sm_motion.subMot; }
        ESubMotionState getSubMotionState(void) const { return sm_motion.subMotState; }
        EAction getAction(void) const { return sm_motion.act; }
        EActionState getActionState(void) const { return sm_motion.actState; }

        double getAngularVelocity(void) { return sm_motion.angularVelocity; }
        bool getPoseStored(void) const { return sm_motion.poseStored; }
        void setPoseStored(bool flag) { sm_motion.poseStored = flag; }
        TMotionPose2D getStartPose(void) const { return sm_motion.startPose; }
        TMotionPose2D getCurPose(void) const { return sm_motion.curPose; }
        double getTimeNeeded(void) { return sm_motion.timeNeeded; }
        void setMotionParams(TMotionParams param);
        TMotionParams getMotionParams(void) { return sm_motion.motParams; }

        /** Remove later.*/
        void updateCurPose(mrpt::kinematics::CVehicleSimul_DiffDriven* robot)
        {
        	sm_motion.curPose.x = robot->getCurrentGTPose().x;
   	 		sm_motion.curPose.y = robot->getCurrentGTPose().y;
    		sm_motion.curPose.phi = robot->getCurrentGTPose().phi;
        }

        /** Calculate the distance between robot and wall in the heading direction.
         * And calculate the distance between robot and wall on right side.
         * @param scan Pointer that points to the laser object.
         * @param winLength How many data points needed to calculate the distance.
         * @return Distance.
         */
        void updateDist(mrpt::obs::CObservation2DRangeScan* scan,
                          size_t winLength);

        /** Frame rotation.
         * This method transfers p0=(x0, y0) in frame 0 to p1=(x1, y1) in frame 1,
         * with phiRad angle difference between frame 0 and frame 1.
         * @param x p0's X axis coordinates.
         * @param y p0's Y axis coordinates.
         * @param phiRad angle difference between frame 0 and frame 1 in rad.
         * Positive means rotate counterc clockwise.
         * @return p1
         */
        TMotionPose2D frameRot(double x, double y, double phiRad);
        /** Store start pose.
         * Used to calculate end pose and verify end pose conditions.
         */
        void storeStartPose(void);
        /** Calculate time needed .
         * Used to calculate end pose and verify end pose conditions.
         */
        void calcTimeNeeded(double velocity, double displacement);

    protected:

    private:
        static TMotion sm_motion;  /**< All shared variables, like velocity ,pose, states, etc.*/
    };
}

#endif /** CACTIONBASE_H_ */
