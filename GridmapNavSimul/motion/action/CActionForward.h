/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file CActionForward.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Forward action
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
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
    /** 
    * Forward \n
    *
    * example: \n
    *
    * #include CActionForward.h \n
    *
    * fwd->forward(1, 90); \n
    * if(fwd->getActionState()==FORWARD_FINISHED) \n
    * { \n
    *     printf("Forwarding task finished. Please command the next order.\n"); \n
    * }
    *
    */
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
