/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file TMotionPoint.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Point struct(only used in path generating)
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef TMOTIONPOINT_H_
#define TMOTIONPOINT_H_

#include <climits>
#include <stdio.h>

namespace motion
{
	/** 
    * Point struct.
    * Member variable x, y, f, g, h and a pointer points parent
    */
	struct TMotionPoint
	{
		int X;
		int Y;
		int F;
		int G;
		int H;
		TMotionPoint* parentPoint;

		TMotionPoint(int x, int y)
		{
			this->X = x;
			this->Y = y;
			this->F = 0;
			this->G = 0;
			this->H = 0;
			parentPoint = NULL;
		}

		void calcF(void) { F = G + H; }
	};
}
#endif /** TMOTIONPOINT_H_ */
