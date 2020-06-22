/*
 * @Author: your name
 * @Date: 2020-06-19 17:52:23
 * @LastEditTime: 2020-06-20 16:00:05
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \CH579\APP\peripheral_main.c
 */
/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : xiaowenlg
* Version            : V1.1
* Date               : 2019/11/05
* GitHubSSH          ：git@github.com:xiaowenlg/CH579.git
* Description        : CH579通过外设中断记录活动次数，时间并计算卡路里，然后通过BLE发送到手机

*******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "CH57x_common.h"
#include "HAL.h"
#include "gattprofile.h"
#include "peripheral.h"
#include "DataProcessing.h"
/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__align(4) u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if (defined(BLE_MAC)) && (BLE_MAC == TRUE)
u8C MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif

/*******************************************************************************
* Function Name  : main
* Description    : 主函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{

  Sleep_Init(); //睡眠配置
  GPIOA_SetBits(bTXD1);
  GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
  GPIOB_ModeCfg(PB15_LED, GPIO_ModeOut_PP_20mA);
  UART1_DefInit();
  Weak_Source_Init();          //唤醒源配置
  sd_t->productID = PRODUCTID; //设置设备id
#if (PRINT_VER_LIB)
  PRINT("%s\n", VER_LIB);
#endif
  CH57X_BLEInit();
  HAL_Init();
  GAPRole_PeripheralInit();
  Peripheral_Init();
  while (1)
  {
    TMOS_SystemProcess();
  }
}

/******************************** endfile @ main ******************************/
