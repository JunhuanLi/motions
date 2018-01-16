#ifndef CACTIONARC_H_
#define CACTIONARC_H_

#include <mrpt/utils/CTicTac.h>
#include "TMotionPose2D.h"
#include "TMotionPIDCon.h"
#include "motionEnums.h"
#include "motionUtils.h"
#include "CActionBase.h"

#define mkturns motion::CActionArc::getInstance()

namespace motion
{
    class CActionArc : public CActionBase
    {
    public:
        static CActionArc* getInstance(void);
        virtual ~CActionArc(void) {}

        void reset(void);

        void setEndPose(TMotionPose2D pose) { m_endPose = pose; }
        TMotionPose2D getEndPose(void) const { return m_endPose; }
        void setStartUp(bool val) { m_startUp = val; }
        bool getStartUp(void) const { return m_startUp; }

        /** Make the vehicle make arc.
         * @param w Linear velocity(rad/s). w is positive.
         * @param angleDegree The desired angle in degree we want the robot rotate.
         *        angleDegree > 0 rotate counterclockwise.
         * @param radius Arc radius.
         */
        void arc(double w, double angDeg, double radius=0);

    protected:

    private:
        CActionArc(void) {}
        CActionArc(const CActionArc&);
        CActionArc& operator=(const CActionArc&);

         /** Calculate the end pose of the robot.
         * @param angRad The desired angle in rad we want the robot rotate.
         * @see calcRem
         */
        void calcEndPose(double angRad);
        /** Calculate the remaining angle.
         * @return How much angle remaining.
         * @see calcEndPose
         */
        double calcRem(void);
        /** Calculate linear velocity by the given angle and radius.
         * @param angDeg The desired angle in degree we want the robot rotate.
         * @param radius Radius of the arc.
         * @see calcRem
         */
        double calcLinVel(double angDeg, double radius);
        /** Calculate inner string length.
         * @param angDeg The desired angle in degree we want the robot rotate.
         * @param radius Radius of the arc.
         */
        void calcInnerStrLen(double angDeg, double radius);
        /** Calculate the yaw by heading direction and target direction.
         * @param angDeg The desired angle in degree we want the robot rotate.c.
         */
        double calcYaw(double angRad);
        /** Verify pose condition after heading condition satisfied.
         * When the robot rotated to the target heading direction, use this method
         * to double check whether the robot reached the target location.
         * @return true if pose verify passed.
         */
        bool poseVerified(void);
        /** If the desired angle is near 360 degree, we need the vehile to start up.
         * @param angleDegree The desired angle in degree we want the robot rotate.
         * @see updateStartUp
         */
        void startUp(double angDeg);

        /** Update the start-up state, reset the state when start-up finished.
         * @see startUp
         */
        void updateStartUp();

        TMotionPose2D m_endPose;  /**< The end pose. */
        bool m_startUp;  /**< Start up mark. */
        double m_innerStrLen;  /**< Inner string length. Used to calculate target location. */
        double m_linVel;  /**< Linear velocity needed to finished the arc task. */
        double m_commandAngDeg;  /**< Command angle in degree. */
        double m_myCurPhi;  /**< Robot current phi which was translated to range 0~360 degree. */
        double m_phiRotated;  /**< Rotated phi in rad. */
        double m_angRem;  /**< Angle remaining which is equal to \sa m_commandAngDeg - rad2Deg(\sa m_phiRotated). */
        bool m_slightTuneOn;  /**< Slightly tune period on mark. */
        bool m_slightTuneFinished;  /**< Slightly tune period finished mark. */
        ///TMotionPIDCon m_controller;
    };

}
#endif /** CACTIONARC_H_ */
