#ifndef CACTIONBACKWARD_H_
#define CACTIONBACKWARD_H_

#include <mrpt/utils/CTicTac.h>
#include "TMotionPose2D.h"
#include "motionEnums.h"
#include "CActionBase.h"
#include "motionUtils.h"

#define bkw motion::CActionBackward::getInstance()

namespace motion
{
    class CActionBackward : public CActionBase
    {
    public:
        static CActionBackward* getInstance(void);
        virtual ~CActionBackward(void) {}

        void reset(void);

        /**
         * Move the robot backward straightly.
         * @param v The backward linear velocity.
         * @param distance The desired distance we want the robot backward.
         */
        void backward(double v, double distance);

    protected:

    private:
        CActionBackward(void) {}
        CActionBackward(const CActionBackward&);
        CActionBackward& operator=(const CActionBackward&);

        /** Calculate the end pose of the robot.
         * @param distance The desired distance in meter we want the robot translate.
         * @see calcRem
         */
        void calcEndPose(double distance);
        /** Calculate the remaining distance.
         * @return How much distance remaining.
         * @see calcEndPose
         */
        double calcRem(void);

        TMotionPose2D m_endPose;
    };

}
#endif /** CACTIONBACKWARD_H_ */

