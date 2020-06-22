/*
 * @Author: your name
 * @Date: 2020-06-20 09:31:45
 * @LastEditTime: 2020-06-22 20:42:12
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \CH579\MCUDriver\HAL\include\BspConfig.h
 * 本次用计算卡路里公式:Cal = (3mtv)/40                  m:体重,t:运动时间,v:运动速度
 */
#ifndef __BSPCONFIG_H
#define __BSPCONFIG_H
#include "CH57x_common.h"

#define SENSOR GPIO_Pin_12               //干簧管引脚 PA12
#define PRODUCTID "00010000400023000002" //产品ID
#define COEFFICIENG                      //卡路里系数
#define WEIGHT 60.00                     //体重
#define PB15_LED GPIO_Pin_15             //led

//debug switch
#define SLEEP_DEBUG 1       //low power debug 1open,0shut
#define EVENT_RUNNING 1     // use this macro to open or close event is running
#define PRINT_VER_LIB 1     //print ver_lib
#define PRINT_BLE_RECEIVE 1 //print ble received message
#define PRINT_BLE_UPDATE 0  //连接更新
//定义数据结构体
typedef struct SportData
{
    uint16_t count;  //活动数据
    uint16_t tim;    //活动时间
    uint16_t cal;    //消耗热量
    uint16_t freq;   //速度
    uint8_t rate;    //心率
    char *productID; //产品ID
} SportData_t;

//low speed measure Enable
#define MEASURE_SPEED 1 //设置采样速度

#endif
