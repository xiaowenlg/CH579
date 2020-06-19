/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.1
* Date               : 2019/11/05
* Description        : ����ӻ�Ӧ��������������ϵͳ��ʼ��
*******************************************************************************/

/******************************************************************************/
/* ͷ�ļ����� */
#include "CONFIG.h"
#include "CH57x_common.h"
#include "HAL.h"
#include "gattprofile.h"
#include "peripheral.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__align(4) u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];
extern uint16 Number_Records; //��¼����
uint16 Count_now = 0;
#if (defined(BLE_MAC)) && (BLE_MAC == TRUE)
u8C MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif
void Sleep_Init() //˯��׼����ʼ��
{
  SetSysClock(CLK_SOURCE_HSE_32MHz); // �����ⲿ32M����Ƶ
  GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
  GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
}

void Weak_Source_Init()
{
  /* ���û���ԴΪ GPIO - PA6&PA5 */
  GPIOA_ModeCfg(GPIO_Pin_6 | GPIO_Pin_5, GPIO_ModeIN_PU);
  GPIOA_ITModeCfg(GPIO_Pin_6 | GPIO_Pin_5, GPIO_ITMode_FallEdge); // �½��ػ���
  NVIC_EnableIRQ(GPIO_IRQn);
  PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE);
}

/*******************************************************************************
* Function Name  : main
* Description    : ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
#ifdef DEBUG
  Sleep_Init(); //˯������
  GPIOA_SetBits(bTXD1);
  GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
  GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeOut_PP_20mA);
  UART1_DefInit();
  Weak_Source_Init(); //����Դ����
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
      Number_Records++; //�����ۼ�
    Count_now++;
    PRINT("Number=%d\n", Number_Records);
  }

  GPIOA_ClearITFlagBit(GPIO_Pin_6 | GPIO_Pin_5);
}
/******************************** endfile @ main ******************************/
