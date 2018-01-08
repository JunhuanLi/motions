#include "CPathGenerator.h"
#include "CMotionNode.h"
#include <vector>
#include "map.h"
#include <cmath>
#include <iostream>

#define OBLIQUE 14
#define STEP 10

using namespace motion;
using namespace std;

CPathGenerator::CPathGenerator(void)
{
}

list<TMotionPoint*> CPathGenerator::calcPath(TMotionPoint* start, 
									  TMotionPoint* target,
									  bool isIgnoreCorner)
{
	openList.push_back(start);
	while(!openList.empty())
	{
		TMotionPoint* tmpStartPoint = findMin();
		openList.remove(tmpStartPoint);
		closeList.push_back(tmpStartPoint);

		auto surroundPoints = surrounding(tmpStartPoint, isIgnoreCorner);
		for(auto curPnt:surroundPoints)
		{

			if(isExist(openList,curPnt))
			{
				//���¼���G�������С���滻���ڵ�Ϊ��ǰ��㣬���¼���GF������ʲô������
				foundPoint(tmpStartPoint, curPnt);
			}
			else
			{
				//��ӵ�openList����Ҽ���FGH
				notFoundPoint(tmpStartPoint, target, curPnt);
			}
		}
		if(isExist(openList, target))
			break;
	}

	list<TMotionPoint*> path;
	TMotionPoint* pnt = isExist(openList, target);
	while(pnt->X != start->X || pnt->Y != start->Y)
	{
		path.push_front(pnt);
		pnt = pnt->parentPoint;
	}
	path.push_front(start);
	
	cout<<"Finished.\n"<<endl;
	return path;
}

vector<TMotionPoint*> CPathGenerator::surrounding(TMotionPoint* tmpStartPoint, bool isIgnoreCorner)
{
	vector<TMotionPoint*> surrPnts;
	
	for(int x = tmpStartPoint->X - 1; x <= tmpStartPoint->X + 1; x++)
	{
		for(int y = tmpStartPoint->Y - 1; y <= tmpStartPoint->Y + 1; y++)
		{
			if(canReach(tmpStartPoint, x, y, isIgnoreCorner))
				surrPnts.push_back(new TMotionPoint(x, y));
		}
	}

	return surrPnts;
}

void CPathGenerator::foundPoint(TMotionPoint* tmpStartPoint, TMotionPoint* curPoint)
{
	int tmpG = calcG(tmpStartPoint, curPoint);
	if(tmpG < curPoint->G)
	{
		curPoint->parentPoint = tmpStartPoint;
		curPoint->G = tmpG;
		curPoint->calcF();
	}
	else
	{
		///do nothing
	}
}

void CPathGenerator::notFoundPoint(TMotionPoint* tmpStartPoint, 
								   TMotionPoint* target, 
								   TMotionPoint* curPoint)
{
	curPoint->parentPoint = tmpStartPoint;
	curPoint->G = calcG(tmpStartPoint, curPoint);
	curPoint->H = calcH(target, curPoint);
	curPoint->calcF();
	openList.push_back(curPoint);
}

bool CPathGenerator::canReach(int x, int y)
{
	return map[x][y] == 0;
}

bool CPathGenerator::canReach(TMotionPoint* tmpStartPoint, int x, int y, bool isIgnoreCorner)
{
	if(!canReach(x, y) || isExist(closeList, &TMotionPoint(x, y)))
		return false;
	/** �� 1 ���Ҳ���closeList*/
	else
	{
		/** x y�����ϡ��¡�����*/
		if(abs(tmpStartPoint->X - x) + abs(tmpStartPoint->Y - y) == 1)
			return true;
		else
		{
			if(canReach(x, tmpStartPoint->Y) && canReach(tmpStartPoint->X, y))
				return true;
			else
				return isIgnoreCorner;
		}
	}
}

int CPathGenerator::calcG(TMotionPoint* tmpStartPoint,TMotionPoint* curPoint)
{
	int G = 0;
	int parentG = 0;

	if(abs(tmpStartPoint->X - curPoint->X) + abs(tmpStartPoint->Y - curPoint->Y) == 2)
		G = OBLIQUE;
	else
		G = STEP;

	bool tmpBool = curPoint->parentPoint == nullptr;
	if(!tmpBool)
	{
		parentG = curPoint->parentPoint->G;
	}
	else
	{
		parentG = 0;
	}
	return G + parentG;
}

int CPathGenerator::calcH(TMotionPoint* target,TMotionPoint* curPoint)
{
	int step = abs(target->X - curPoint->X) + abs(target->X - curPoint->X);
	return step * STEP;
}

TMotionPoint* CPathGenerator::findMin(void)
{
	if(!openList.empty())
	{
		auto resPoint = openList.front();
		for(auto &point:openList)
		{
			if(point->F<resPoint->F)
				resPoint = point;
		}
		return resPoint;
	}
	return NULL;
}

TMotionPoint* CPathGenerator::isExist(list<TMotionPoint*> &list, TMotionPoint* point)
{
	for(auto p:list)
		if(p->X == point->X && p->Y == point->Y)
			return p;
	return NULL;
}