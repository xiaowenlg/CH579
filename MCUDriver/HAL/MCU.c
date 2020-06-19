/********************************** (C) COPYRIGHT *******************************
* File Name          : MCU.c
* Author             : WCH
* Version            : V1.1
* Date               : 2019/11/05
* Description        : Ӳ������������BLE��Ӳ����ʼ��
*******************************************************************************/

/******************************************************************************/
/* ͷ�ļ����� */
#include "CH57x_common.h"
#include "HAL.h"
#include "config.h"

tmosTaskID halTaskID;
uint8 flag = 0, Led_Num = 0;
extern int8 rssi_t;
/*******************************************************************************
 * @fn          CH57X_BLEInit
 *
 * @brief       BLE ���ʼ��
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void CH57X_BLEInit(void)
{
  uint8 i;
  bleConfig_t cfg;

  if (tmos_memcmp(VER_LIB, VER_FILE, strlen(VER_FILE)) == FALSE)
  {
    PRINT("head file error...\n");
    while (1)
      ;
  }
  R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
  R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
  R16_CLK_SYS_CFG = RB_CLK_OSC32M_XT | (2 << 6) | 0x08; // 32M -> Fsys
  R8_SAFE_ACCESS_SIG = 0;
  SysTick_Config(SysTick_LOAD_RELOAD_Msk);
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; /* disable SysTick IRQ */
  tmos_memset(&cfg, 0, sizeof(bleConfig_t));
  cfg.MEMAddr = (u32)MEM_BUF;
  cfg.MEMLen = (u32)BLE_MEMHEAP_SIZE;
  cfg.BufMaxLen = (u32)BLE_BUFF_MAX_LEN;
  cfg.BufNumber = (u32)BLE_BUFF_NUM;
  cfg.TxNumEvent = (u32)BLE_TX_NUM_EVENT;
  cfg.TxPower = (u32)BLE_TX_POWER;
#if (defined(BLE_SNV)) && (BLE_SNV == TRUE)
  cfg.SNVAddr = (u32)BLE_SNV_ADDR;
#endif
#if (CLK_OSC32K)
  cfg.SelRTCClock = (u32)CLK_OSC32K;
#endif
  cfg.ConnectNumber = (PERIPHERAL_MAX_CONNECTION & 3) | (CENTRAL_MAX_CONNECTION << 2);
  cfg.srandCB = SYS_GetSysTickCnt;
#if (defined TEM_SAMPLE) && (TEM_SAMPLE == TRUE)
  cfg.tsCB = HAL_GetInterTempValue;
#if (CLK_OSC32K)
  cfg.rcCB = Calibration_LSI; // �ڲ�32Kʱ��У׼
#endif
#endif
#if (defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
  cfg.WakeUpTime = WAKE_UP_RTC_MAX_TIME;
  cfg.sleepCB = CH57X_LowPower; // ����˯��
#endif
#if (defined(BLE_MAC)) && (BLE_MAC == TRUE)
  for (i = 0; i < 6; i++)
    cfg.MacAddr[i] = MacAddr[5 - i];
#endif
  if (!cfg.MEMAddr || cfg.MEMLen < 4 * 1024)
    while (1)
      ;
#if (defined HAL_SLEEP) && (HAL_SLEEP == TRUE)
  if ((u32)MEM_BUF < (u32)0x20003800)
  {
    PRINT("RAM config error...\n");
    while (1)
      ;
  }
#endif
  i = BLE_LibInit(&cfg);
  if (i)
  {
    PRINT("LIB init error code: %x ...\n", i);
    while (1)
      ;
  }
}
void GotoCutPower()
{
  //˯��
  PRINT("shut down by rssi_t \n");
  GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
  GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
  DelayMs(1);
  LowPower_Shutdown(NULL); //ȫ���ϵ磬���Ѻ�λ
  /* ��ģʽ���Ѻ��ִ�и�λ������������벻������ */
  SetSysClock(CLK_SOURCE_HSE_32MHz); // �л���ԭʼʱ��
  PRINT("wake.. \n");
  DelayMs(500);
}
/*******************************************************************************
 * @fn          HAL_ProcessEvent
 *
 * @brief       Ӳ����������
 *
 * input parameters
 *
 * @param       task_id.
 * @param       events.
 *
 * output parameters
 *
 * @param       events.
 *
 * @return      None.
 */
tmosEvents HAL_ProcessEvent(tmosTaskID task_id, tmosEvents events)
{
  uint8 *msgPtr;
  uint8 Freq = 0;
  if (events & SYS_EVENT_MSG)
  { // ����HAL����Ϣ������tmos_msg_receive��ȡ��Ϣ��������ɺ�ɾ����Ϣ��
    msgPtr = tmos_msg_receive(task_id);
    if (msgPtr)
    {
      /* De-allocate */
      tmos_msg_deallocate(msgPtr);
    }
    return events ^ SYS_EVENT_MSG;
  }
  if (events & LED_BLINK_EVENT)
  {
#if (defined HAL_LED) && (HAL_LED == TRUE)
    HalLedUpdate();
#endif // HAL_LED
    return events ^ LED_BLINK_EVENT;
  }
  if (events & HAL_KEY_EVENT)
  {
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
    HAL_KeyPoll(); /* Check for keys */
    if (!Hal_KeyIntEnable)
    {
      tmos_start_task(halTaskID, HAL_KEY_EVENT, MS1_TO_SYSTEM_TIME(100));
    }
#endif
  }
  if (events & HAL_TEST_EVENT)
  {
    static uint8 tim = 0;
    flag = ~flag;
    if (flag)
      GPIOA_SetBits(GPIO_Pin_15);
    else
      GPIOA_ResetBits(GPIO_Pin_15);
    PRINT("Led_Num=%d   riss_t=%d\n", Led_Num++, rssi_t);
    if (Led_Num > 50)
      Led_Num = 0;
    Freq = Count_now;
    Count_now = 0;
    if (Freq == 0)
    {
      /* code */
      if (tim++ > 20)
      {
        //GotoCutPower();
      }
    }
    else
    {
      tim = 0;
    }

    if (rssi_t < -90)
    {
      // GotoCutPower();
    }
    tmos_start_task(halTaskID, HAL_TEST_EVENT, MS1_TO_SYSTEM_TIME(1000));
    return events ^ HAL_TEST_EVENT;
  }
  return 0;
}

/*******************************************************************************
 * @fn          HAL_Init
 *
 * @brief       Ӳ����ʼ��
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void HAL_Init() //Ӧ�ò��ʼ��
{
  halTaskID = TMOS_ProcessEventRegister(HAL_ProcessEvent); //TMOS ����ص�����            //Ӧ�ò�����ص�����
  HAL_TimeInit();
#if (defined HAL_SLEEP) && (HAL_SLEEP == TRUE)
  HAL_SleepInit();
#endif
#if (defined HAL_LED) && (HAL_LED == TRUE)
  HAL_LedInit();
#endif
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
  HAL_KeyInit();
#endif
  __enable_irq();
  tmos_start_task(halTaskID, HAL_TEST_EVENT, 1000); // ���һ����������
}

/*******************************************************************************
 * @fn          LLE_IRQHandler
 *
 * @brief       LLE interrupt function 
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void LLE_IRQHandler(void)
{
  BLE_IRQHandler();
}

/*******************************************************************************
 * @fn          HAL_GetInterTempValue
 *
 * @brief       None.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
uint16 HAL_GetInterTempValue(void)
{
  uint8 sensor, channel, config;
  uint16 adc_data;

  sensor = R8_TEM_SENSOR;
  channel = R8_ADC_CHANNEL;
  config = R8_ADC_CFG;
  R8_TEM_SENSOR |= RB_TEM_SEN_PWR_ON;
  R8_ADC_CHANNEL = CH_INTE_VTEMP;
  R8_ADC_CFG = RB_ADC_POWER_ON | (2 << 4);
  R8_ADC_CONVERT |= RB_ADC_START;
  while (R8_ADC_CONVERT & RB_ADC_START)
    ;
  adc_data = R16_ADC_DATA;
  R8_TEM_SENSOR = sensor;
  R8_ADC_CHANNEL = channel;
  R8_ADC_CFG = config;
  return (adc_data);
}

/******************************** endfile @ mcu ******************************/
