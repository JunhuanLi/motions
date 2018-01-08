#pragma once

#include "TMotionPoint.h"
#include "CMotionList.h"
#include <stdio.h>
#include <vector>
#include <list>

using namespace std;

namespace motion
{
	class CPathGenerator
	{
	public:
		CPathGenerator(void);
		virtual ~CPathGenerator(void) {}

		list<TMotionPoint*> calcPath(TMotionPoint* start, 
							         TMotionPoint* target,
									 bool isIgnoreCorner);
		bool compF(const TMotionPoint& point1, const TMotionPoint& point2);
		vector<TMotionPoint*> surrounding(TMotionPoint* curPoint, bool isIgnoreCorner);
		void foundPoint(TMotionPoint* tmpStartPoint, TMotionPoint* curPoint);
		void notFoundPoint(TMotionPoint* tmpStartPoint, TMotionPoint* target, TMotionPoint* curPoint);
		bool canReach(TMotionPoint* curPoint, int x, int y, bool isIgnoreCorner);
		bool canReach(int x, int y);
		int calcG(TMotionPoint* tmpStartPoint,TMotionPoint* curPoint);
		int calcH(TMotionPoint* target,TMotionPoint* curPoint);

		TMotionPoint* findMin(void);  /**< Found minimum value F in list. */
		TMotionPoint* isExist(list<TMotionPoint*> &list, TMotionPoint* point);

	private:
		list<TMotionPoint*> openList;
		list<TMotionPoint*> closeList;
	};
}
