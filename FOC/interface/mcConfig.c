/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-07-04 09:16:17
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-08-06 16:31:54
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\interface\mcConfig.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "mcConfig.h"
#include "main.h"
#include "arm_math.h"
#include "mcMath.h"

const uint8_t frameHeader[] = {'T','T',':'};
const uint8_t frameEnd[] = {'E','T','!'};

FrameForRTT frameForRTT;
FrameReceiveForUSART frameReceiveForUSART;

void resetFrameReceiveForUSART(void)
{
    for(int i = 0;i < sizeof(FrameReceiveForUSART);i++){
        frameReceiveForUSART.receiveDat[i] = '\0';
    }
}

FrameSendForUSART frameSendForUSART = {
    .tail = {0x00,000,0x80,0x7f}
};

void Hardware_SafatyTaskEvent(void)
{
    static uint32_t timeBase = 0u;
    //1s温度采集任务
    if((timeBase % 1000) == 0u){
        pSens->busAndTemp.com2 = Hardware_GetTemperature();
        if(pSens->busAndTemp.com2 > 65.f){
            Hardware_StopPWM();
        }
        LL_GPIO_TogglePin(GPIOC,LL_GPIO_PIN_10);
    }
    
    //1ms解析一次上位机命令
    static uint16_t index = 0u;
    uint16_t index1 = index % sizeof(frameReceiveForUSART);
    uint16_t index2 = (index + 1) % sizeof(frameReceiveForUSART);
    uint16_t index3 = (index + 2) % sizeof(frameReceiveForUSART);
    uint16_t index4 = (index + 3) % sizeof(frameReceiveForUSART);
    if(frameReceiveForUSART.receiveDat[index1] == 'S' && frameReceiveForUSART.receiveDat[index2] == ':' && frameReceiveForUSART.receiveDat[index4] == '\n'){
        uint8_t receiveDat = frameReceiveForUSART.receiveDat[index3];
        if(receiveDat < 0xC9){
            pSpPIC->target = 2.f * MATH_PI * receiveDat;
        }else{
            pSpPIC->target = -pSpPIC->target;
        }
        frameReceiveForUSART.receiveDat[index1] = frameReceiveForUSART.receiveDat[index2] = '\0';
        frameReceiveForUSART.receiveDat[index3] = frameReceiveForUSART.receiveDat[index4] = '\0';
    }

    index++;
    timeBase++;
}

void Hardware_PerformanceTaskEvent(void)
{
    // static uint16_t cnt = 0u;
    // frameForRTT.dat0 = pSens->currentAB.com1;
    // frameForRTT.dat1 = 4.f;
    // frameForRTT.dat2 = 4.f;
    // if(cnt == 0u){
    //     cnt++;
    //     SEGGER_RTT_WriteNoLock(0,frameHeader,3);
    //     SEGGER_RTT_WriteNoLock(0,&frameForRTT,sizeof(frameForRTT));
    // }
    // else if(cnt == 999u){
    //     cnt = 0u;
    //     SEGGER_RTT_WriteNoLock(0,&frameForRTT,sizeof(frameForRTT));
    //     SEGGER_RTT_WriteNoLock(0,frameEnd,3);
    // }
    // else{
    //     cnt++;
    //     SEGGER_RTT_WriteNoLock(0,&frameForRTT,sizeof(frameForRTT));
    // }
    if(LL_DMA_GetDataLength(DMA1,LL_DMA_CHANNEL_1) == 0u){
        frameSendForUSART.dat0 = pSens->currentAB.com1;
        frameSendForUSART.dat1 = pHFPI->response_HF_iDQ.com1;
        frameSendForUSART.dat2 = pHFPI->response_HF_iDQ.com2;
        LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_1);
        LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1,sizeof(FrameSendForUSART));
        LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);
    }
}
void Hardware_Init(void)
{

}
bool Hardware_MCStartOrStop(void)
{
    static bool isStart = false;
    static uint16_t key3Cnt = 0u;
    uint32_t key3Signal = LL_GPIO_ReadInputPort(GPIOC) & (LL_GPIO_PIN_3);
    if(key3Signal){
        key3Cnt = 0u;
    }
    else{
      if(++key3Cnt > 300u){
        key3Cnt = 0;
        isStart = !isStart;
      }
    }
    return isStart;
}
void Hardware_StartPWM(void)
{
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3N);
    LL_TIM_EnableAllOutputs(TIM1);
    LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_11);
}

void Hardware_StopPWM(void)
{
    LL_TIM_DisableAllOutputs(TIM1);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH3N);
    LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_11);
}
void Hardware_ForceSwitchOnAllLowSides(void)
{
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH1,LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH2,LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH3,LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH1N,LL_TIM_OCMODE_FORCED_ACTIVE);
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH2N,LL_TIM_OCMODE_FORCED_ACTIVE);
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH3N,LL_TIM_OCMODE_FORCED_ACTIVE);
}

void Hardware_ResetAllLowSides(void)
{
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH1,LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH2,LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH3,LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH1N,LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH2N,LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH3N,LL_TIM_OCMODE_PWM2);
}

void Hardware_SetCCR(int32_t ccr1,int32_t ccr2,int32_t ccr3)
{
    LL_TIM_OC_SetCompareCH1(TIM1,ccr1);
    LL_TIM_OC_SetCompareCH2(TIM1,ccr2);
    LL_TIM_OC_SetCompareCH3(TIM1,ccr3); 
}

Components2 Hardware_GetSinCosVal(f32_t angleRad)
{
    Components2 sinCos;

    int32_t angleFixed = (angleRad / 3.141592653589793f * 2147483648.f);

    LL_CORDIC_WriteData(CORDIC,angleFixed);

    int32_t sinFixed = LL_CORDIC_ReadData(CORDIC);
    int32_t cosFixed = LL_CORDIC_ReadData(CORDIC);

    sinCos.com1 = sinFixed / 2147483648.f;
    sinCos.com2 = cosFixed / 2147483648.f;

    // sinCos.com1 =  arm_sin_f32(angleRad);
    // sinCos.com2 = arm_cos_f32(angleRad);
    
    return sinCos;
}
f32_t Hardware_FastSquareRoot(f32_t x)
{
    f32_t out;
    arm_sqrt_f32(x,&out);
    return out;
}
f32_t Hardware_FastReciprocalSquareRoot(f32_t x)
{
    int32_t i;
    f32_t x2,y;
    x2 = x * 0.5f;
    y = x;
    i = *(int32_t*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(f32_t*)&i;
    y = y * (1.5f - (x2 * y * y));
    return y;
}
Components2 Harware_GetCurrentAB(void)
{
    Components2 ab;
    ab.com1 = (1.635f - ((float)LL_ADC_INJ_ReadConversionData12(ADC1,LL_ADC_INJ_RANK_1) / 65535.f * 3.27f)) / 20.f * 200.f;
    ab.com2 = (1.635f - ((float)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_1) / 65535.f * 3.27f)) / 20.f * 200.f;
    return ab;
}

f32_t Hardware_GetBusVoltage(void)
{
    f32_t bus;
    LL_ADC_REG_StartConversion(ADC1);
    while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0u);
    bus = (float)LL_ADC_REG_ReadConversionData12(ADC1) / 65535.f * 3.27f * 101.f;
    while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0u);
    pSens->adcCorrectionCoefficient = 1.21142578125f / ((float)LL_ADC_REG_ReadConversionData12(ADC1) / 65535.f * 3.27f);
    return bus;
}

f32_t Hardware_GetTemperature(void)
{
    LL_I2C_GenerateStartCondition(I2C1);
    int16_t dat = ((tempDat) >> 8)|((tempDat) << 8);
    dat >>= 5;
    return (float)dat * 0.125f;
}


void Hardware_SetPulseCounter(uint32_t cnt)
{
    LL_TIM_SetCounter(TIM17,cnt);
}
uint32_t Hardware_GetPulseCounter(void)
{
    return LL_TIM_GetCounter(TIM17);
}
void Hardwarre_SetABZCounter(uint32_t cnt)
{
    LL_TIM_SetCounter(TIM8,cnt);
}

uint32_t Hardware_GetABZCounter(void)
{
    return LL_TIM_GetCounter(TIM8);
}

uint32_t Hardware_GetABZCounterDir(void)
{   
    return (LL_TIM_GetDirection(TIM8) == LL_TIM_COUNTERDIRECTION_UP) ? 1u : 0u;
}

