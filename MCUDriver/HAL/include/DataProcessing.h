/*
 * @Author: your name
 * @Date: 2020-06-20 10:49:04
 * @LastEditTime: 2020-06-20 16:03:17
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \CH579\MCUDriver\HAL\include\DataProcecssing.h
 * 数据处理
 */
#ifndef __DATAPROCESSING_H
#define __DATAPROCESSING_H
#include "stdint.h"

uint16_t ConsumeHeat(float weight, float tim, float v); //计算热量
void Sleep_Init(void);                                      //睡眠初始化
void Weak_Source_Init(void);                                //睡眠初始化

#endif

