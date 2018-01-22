/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file main.cpp
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Path generator demo
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#include "CPathGenerator.h"
#include "CMotionList.h"
#include "TMotionPoint.h"
#include <iostream>

using namespace std;
using namespace motion;

//void main(void)
//{
//	CPathGenerator* pg = new CPathGenerator();
//	TMotionPoint start(1, 1);
//	TMotionPoint end(8, 8);
//	auto path = pg->calcPath(&start, &end, false);
//
//	while(!path.empty())
//	{
//		cout<<"("<<path.front()->X<<", "<<path.front()->Y<<")"<<endl;
//		path.pop_front();
//	}
//	system("pause");
//}