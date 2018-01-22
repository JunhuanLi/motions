/**
  * Copyright (C), 1996-2017, TOPBAND. Co., Ltd. \n
  * All right reserved.
  *
  * @file map.h
  * @author Junhuan Li       
  * @version v1.0      
  * @date 18/01/17
  * @brief Temporary defined map(used to test algorithm)
  * @note 
  * 1. --- \n
  * History: Create this file \n
  * <author>       <time>   <version >      <desc> \n
  * Junhuan Li    18/01/17     1.0         create file
  */
#ifndef MAP_H_
#define MAP_H_

#define MAP_COL 10
#define MAP_ROW 10

int map[MAP_ROW][MAP_COL] = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
              							 {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
              							 {1, 0, 0, 0, 1, 0, 1, 0, 0, 1},
              							 {1, 0, 0, 1, 1, 1, 1, 1, 0, 1},
              							 {1, 0, 0, 1, 1, 1, 1, 1, 0, 1},
              							 {1, 0, 0, 1, 1, 0, 0, 1, 0, 1},
              							 {1, 1, 0, 1, 0, 0, 0, 0, 0, 1},
              							 {1, 0, 1, 0, 0, 0, 0, 0, 0, 1},
              							 {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
              							 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
#endif /** MAP_H_ */