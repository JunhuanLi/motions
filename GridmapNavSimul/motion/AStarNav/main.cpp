#include "CPathGenerator.h"
#include "CMotionList.h"
#include "TMotionPoint.h"
#include <iostream>

using namespace std;
using namespace motion;

void main(void)
{
	CPathGenerator* pg = new CPathGenerator();
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