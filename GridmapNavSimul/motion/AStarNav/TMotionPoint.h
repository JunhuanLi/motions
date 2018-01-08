#pragma once

#include <climits>
#include <stdio.h>

namespace motion
{
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
