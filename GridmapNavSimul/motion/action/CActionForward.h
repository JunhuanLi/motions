#ifndef CACTIONFORWARD_H_
#define CACTIONFORWARD_H_

#include <mrpt/utils/CTicTac.h>
#include "TMotionPose2D.h"
#include "motionEnums.h"
#include "CActionBase.h"
#include "motionUtils.h"

#define fwd motion::CActionForward::getInstance()

namespace motion
{
    class CActionForward : public CActionBase
    {
    public:
        static CActionForward* getInstance(void);
        virtual ~CActionForward(void) {}

        void reset(void);

        /**
         * Move the robot forward straightly.
         * @param v is the forward linear velocity.
         * @param distance is the desired distance we want the robot forward.
         */
        void forward(double v, double distance);

    protected:

    private:
        CActionForward(void) {}
        CActionForward(const CActionForward&);
        CActionForward& operator=(const CActionForward&);

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
#endif /** CACTIONFORWARD_H_ */
