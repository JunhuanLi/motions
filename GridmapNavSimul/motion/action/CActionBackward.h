/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CActionBackward.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Backward action
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
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
  /** 
    * Backward \n
    *
    * example: \n
    *
    * #include CActionBackward.h \n
    *
    * bkw->backward(1, 90); \n
    * if(bkw->getActionState()==BACKWARD_FINISHED) \n
    * { \n
    *     printf("Backing task finished. Please command the next order.\n"); \n
    * }
    *
    */
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

