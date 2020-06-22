/*
 * @Author: your name
 * @Date: 2020-06-19 17:52:23
 * @LastEditTime: 2020-06-20 14:02:06
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \CH579\APP\include\peripheral.h
 */
/********************************** (C) COPYRIGHT *******************************
* File Name          : peripheral.h
* Author             : WCH
* Version            : V1.0
* Date               : 2018/12/11
* Description        : 
            
*******************************************************************************/

#ifndef PERIPHERAL_H
#define PERIPHERAL_H
#include "BspConfig.h"
#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Peripheral Task Events
#define SBP_START_DEVICE_EVT 0x0001 //设备开始事件
#define SBP_PERIODIC_EVT 0x0002
#define SBP_READ_RSSI_EVT 0x0004
#define SBP_PARAM_UPDATE_EVT 0x0008
#define SBP_PARAM_UART 0x000a

  /*********************************************************************
 * MACROS
 */
  typedef struct
  {
    uint16 connHandle; // Connection handle of current connection
    uint16 connInterval;
    uint16 connSlaveLatency;
    uint16 connTimeout;
  } peripheralConnItem_t;

  /*********************************************************************
 * FUNCTIONS
 */

  /*
 * Task Initialization for the BLE Application
 */
  extern void Peripheral_Init(void);

  /*
 * Task Event Processor for the BLE Application
 */
  extern uint16 Peripheral_ProcessEvent(uint8 task_id, uint16 events);

  /*********************************************************************
*********************************************************************/
  //extern uint16 Number_Records; //记录次数
  extern SportData_t *sd_t;
#ifdef __cplusplus
}
#endif

#endif
