/*
 * @Author: your name
 * @Date: 2020-06-20 10:58:44
 * @LastEditTime: 2020-06-22 20:01:55
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \CH579\MCUDriver\HAL\DataProcessing.c
 */
#include "CH57x_common.h"
#include "DataProcessing.h"
#include "BspConfig.h"
/*
 * 计算热量 Cal = (3mtv)/40  
 */
uint16_t ConsumeHeat(float weight, float tim, float v)
{
    return (uint16_t)(3 * weight * tim * v) / 40;
}

//Turn off the Power
void GotoCutPower()
{
//睡眠
#if (SLEEP_DEBUG)
    PRINT("shut down by rssi_t \n");
#endif
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    DelayMs(1);
    LowPower_Shutdown(NULL); //全部断电，唤醒后复位
    /* 此模式唤醒后会执行复位，所以下面代码不会运行 */
    SetSysClock(CLK_SOURCE_HSE_32MHz); // 切换到原始时钟
#if (SLEEP_DEBUG)
    PRINT("wake.. \n");
#endif
    DelayMs(500);
}
void Sleep_Init(void) //睡眠准备初始化
{
    SetSysClock(CLK_SOURCE_HSE_32MHz); // 设置外部32M做主频
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
}

void Weak_Source_Init(void)
{
    /* 配置唤醒源为 GPIO - PA12 */
    GPIOA_ModeCfg(SENSOR, GPIO_ModeIN_PU);
    GPIOA_ITModeCfg(SENSOR, GPIO_ITMode_FallEdge); // 下降沿唤醒
    NVIC_EnableIRQ(GPIO_IRQn);
    PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE);
}
