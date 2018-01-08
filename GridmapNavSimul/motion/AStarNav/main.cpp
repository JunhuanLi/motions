#include "CPathGenerator.h"
#include "CMotionList.h"
#include "TMotionPoint.h"
#include <iostream>

using namespace std;
using namespace motion;

void main(void)
{
	CPathGenerator* pg = new CPathGenerator();
	//CMotionList tmpList;
	//tmpList.insertFront(TMotionPoint(1, 0));
	//tmpList.insertFront(TMotionPoint(2, 0));
	//tmpList.insertFront(TMotionPoint(3, 1));
	//tmpList.insertFront(TMotionPoint(5, 5));
	//tmpList.insertFront(TMotionPoint(5, 6));
	TMotionPoint start(1, 1);
	TMotionPoint end(8, 8);
	auto path = pg->calcPath(&start, &end, false);

	while(!path.empty())
	{
		cout<<"("<<path.front()->X<<", "<<path.front()->Y<<")"<<endl;
		path.pop_front();
	}
	system("pause");
}