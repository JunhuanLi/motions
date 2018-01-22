/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file TMotionVector2D.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Vector struct
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef TMOTIONVECTOR2D_H_
#define TMOTIONVECTOR2D_H_

#include <cmath>
#include <cstdio>

namespace motion
{
   /** 
    * Motion vector \n
    *
    */
    struct TMotionVector2D
    {
        double x;
        double y;

        TMotionVector2D():x(0.0), y(0.0) {}
        TMotionVector2D(double xx, double yy):x(xx), y(yy) {}

        inline const TMotionVector2D& operator=(const TMotionVector2D& rhs)
        {
            x = rhs.x;
            y = rhs.y;
            return *this;
        }

        inline TMotionVector2D operator+(const TMotionVector2D& rhs)
        {
            return TMotionVector2D(x + rhs.x, y + rhs.y);
        }

        inline TMotionVector2D operator-(const TMotionVector2D& rhs)
        {
            return TMotionVector2D(x - rhs.x, y - rhs.y);
        }

        inline void reset(void) { x = 0.0; y = 0.0; }

        inline void normVec(void)
        {
            double rRoot = sqrt(x * x + y * y);
            x /= rRoot;
            y /= rRoot;
        }

        /** Calculate the dot product.
         * @param vecX A vector.
         * @return dot product
         */
        double dotProduct(TMotionVector2D vecX)
        {
            this->normVec();
            vecX.normVec();
///            double tmp = vecX.x*vecX.x + vecX.y*vecX.y;
///            if( tmp != 1)
///                printf("[ljh] Normalise error occurred. %.3f\n", tmp);
            double xx = this->x * vecX.x;
            double yy = this->y * vecX.y;
            double res = xx + yy;
            if(res > 1) res = 1;
            else if(res < -1) res = -1;
            ///printf("dp: %f\n", res);
            return res;
        }

        /** Calculate the cross product.
         * @param vecX A vector.
         * @return cross product
         */
        inline double crossProduct(TMotionVector2D vecX)
        {
            this->normVec();
            vecX.normVec();
            double xx = this->x * vecX.y;
            double yy = this->y * vecX.x;
            double res = xx - yy;
            if(res >= 1) res = 1;
            else if(res <= -1) res = -1;
            return res;
        }
    };
}

#endif /** TMOTIONVECTOR2D_H_ */
