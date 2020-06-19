/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.1
* Date               : 2019/11/05
* Description        : 外设从机应用主函数及任务系统初始化
*******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "CH57x_common.h"
#include "HAL.h"
#include "gattprofile.h"
#include "peripheral.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__align(4) u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];
extern uint16 Number_Records; //记录次数
uint16 Count_now = 0;
#if (defined(BLE_MAC)) && (BLE_MAC == TRUE)
u8C MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif
void Sleep_Init() //睡眠准备初始化
{
  SetSysClock(CLK_SOURCE_HSE_32MHz); // 设置外部32M做主频
  GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
  GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
}

void Weak_Source_Init()
{
  /* 配置唤醒源为 GPIO - PA6&PA5 */
  GPIOA_ModeCfg(GPIO_Pin_6 | GPIO_Pin_5, GPIO_ModeIN_PU);
  GPIOA_ITModeCfg(GPIO_Pin_6 | GPIO_Pin_5, GPIO_ITMode_FallEdge); // 下降沿唤醒
  NVIC_EnableIRQ(GPIO_IRQn);
  PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE);
}

/*******************************************************************************
* Function Name  : main
* Description    : 主函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
#ifdef DEBUG
  Sleep_Init(); //睡眠配置
  GPIOA_SetBits(bTXD1);
  GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
  GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeOut_PP_20mA);
  UART1_DefInit();
  Weak_Source_Init(); //唤醒源配置
#endif
  PRINT("%s\n", VER_LIB);
  CH57X_BLEInit();
  HAL_Init();
  GAPRole_PeripheralInit();
  Peripheral_Init();
  while (1)
  {
    TMOS_SystemProcess();
  }
}
void GPIO_IRQHandler(void)
{
  if (GPIOA_ReadITFlagBit(GPIO_Pin_5))
  {
    DelayMs(10);
    if (GPIOA_ReadPortPin(GPIO_Pin_5) == 0)
      Number_Records++; //次数累加
    Count_now++;
    PRINT("Number=%d\n", Number_Records);
  }

  GPIOA_ClearITFlagBit(GPIO_Pin_6 | GPIO_Pin_5);
}
/******************************** endfile @ main ******************************/
