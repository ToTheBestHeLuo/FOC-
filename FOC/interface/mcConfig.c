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
    //1.s温度采集任务
    if((timeBase % 1000) == 0u){
        pSens->busAndTemp.com2 = Hardware_GetTemperature();
        if(pSens->busAndTemp.com2 > 65.f){
            Hardware_StopPWM();
        }
    }
    //1s系统运行状态的指示
    if((timeBase % 1000) == 0u){
        LL_GPIO_TogglePin(GPIOC,LL_GPIO_PIN_10);
    }
    
    //1ms解析一次上位机命令
    static uint16_t index = 0u;
    uint16_t index1 = index % sizeof(frameReceiveForUSART);
    uint16_t index2 = (index + 1) % sizeof(frameReceiveForUSART);
    uint16_t index3 = (index + 2) % sizeof(frameReceiveForUSART);
    uint16_t index4 = (index + 3) % sizeof(frameReceiveForUSART);
    if(frameReceiveForUSART.receiveDat[index1] == 'S' && frameReceiveForUSART.receiveDat[index2] == ':' && frameReceiveForUSART.receiveDat[index4] == '\n'){
        pSpPIC->target = 2 * MATH_PI * frameReceiveForUSART.receiveDat[index3];
        frameReceiveForUSART.receiveDat[index1] = frameReceiveForUSART.receiveDat[index2] = '\0';
        frameReceiveForUSART.receiveDat[index3] = frameReceiveForUSART.receiveDat[index4] = '\0';
    }

    //1ms传输一次数据到上位机
    if(LL_DMA_GetDataLength(DMA1,LL_DMA_CHANNEL_1) == 0u){
        frameSendForUSART.dat0 = pSpPIC->target;
        frameSendForUSART.dat1 = pIncABZ->realEleSpeed;
        LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_1);
        LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1,sizeof(FrameSendForUSART));
        LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);
    }

    index++;
    timeBase++;
}

void Hardware_PerformanceTaskEvent(void)
{
    // static uint16_t cnt = 0u;
    // frameForRTT.dat0 = pSens->currentDQ.com1;
    // frameForRTT.dat1 = pSens->currentDQ.com2;
    // frameForRTT.dat2 = pSens->currentAB.com1;
    // frameForRTT.dat3 = pParmeterIndentify->mc_Rs;
    // frameForRTT.dat4 = pParmeterIndentify->mc_Ls;
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
    ab.com1 = (1.65f - ((float)LL_ADC_INJ_ReadConversionData12(ADC1,LL_ADC_INJ_RANK_1) / 65535.f * 3.3f)) / 20.f * 200.f;
    ab.com2 = (1.65f - ((float)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_1) / 65535.f * 3.3f)) / 20.f * 200.f;
    return ab;
}

Components2 Hardware_GetCurrentOffset(void)
{
    Components2 offset;
    offset.com1 = (1.65f - ((float)LL_ADC_INJ_ReadConversionData12(ADC1,LL_ADC_INJ_RANK_1) / 65535.f * 3.3f)) / 20.f * 200.f;
    offset.com2 = (1.65f - ((float)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_1) / 65535.f * 3.3f)) / 20.f * 200.f;
    return offset;
}

f32_t Hardware_GetBusVoltage(void)
{
    f32_t bus;
    LL_ADC_REG_StartConversion(ADC1);
    while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0u);
    bus = (float)LL_ADC_REG_ReadConversionData12(ADC1) / 65535.f * 3.3f * 101.f;
    while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0u);
    pSens->adcCorrectionCoefficient = 1.21142578125f / ((float)LL_ADC_REG_ReadConversionData12(ADC1) / 65535.f * 3.3f);
    return bus;
}
uint32_t Hardware_GetIncABZEncoderTimCnt(void)
{
    return LL_TIM_GetCounter(TIM8);
}
void Hardware_SetIncABZEncoderTimCnt(int32_t cnt)
{
    LL_TIM_SetCounter(TIM8,cnt);   
}

f32_t Hardware_GetTemperature(void)
{
    LL_I2C_GenerateStartCondition(I2C1);
    int16_t dat = ((tempDat) >> 8)|((tempDat) << 8);
    dat >>= 5;
    return (float)dat * 0.125f;
}

f32_t Hardware_GetEleSpeed(void)
{
    static f32_t eleSpeed = 0.f;

    uint32_t lastEncoderCnt = pIncABZ->lastEncoderCnt;
    uint32_t nowEncoderCnt = LL_TIM_GetCounter(TIM8);
    int32_t difEncoderCnt;

    if(LL_TIM_GetDirection(TIM8) == LL_TIM_COUNTERDIRECTION_UP && pIncABZ->dirLPF++ == 10){
        pIncABZ->motorRunSta = 1;
        pIncABZ->dirLPF = 0;
    }
    else if(LL_TIM_GetDirection(TIM8) == LL_TIM_COUNTERDIRECTION_DOWN && pIncABZ->dirLPF-- == -10){
        pIncABZ->motorRunSta = 0;
        pIncABZ->dirLPF = 0;
    }

    if(pIncABZ->zeroPassABZCnt){
        if(pIncABZ->motorRunSta == 1){
            difEncoderCnt = (int32_t)(incABZHandler.encoderPPR_XX_Uint + nowEncoderCnt - lastEncoderCnt);
        }
        else if(pIncABZ->motorRunSta == 0){
            difEncoderCnt = (int32_t)(nowEncoderCnt - lastEncoderCnt - incABZHandler.encoderPPR_XX_Uint);
        }
        pIncABZ->zeroPassABZCnt = 0u;
    }
    else{
        difEncoderCnt = nowEncoderCnt - lastEncoderCnt;
    }

    pIncABZ->lastEncoderCnt = nowEncoderCnt;
    eleSpeed = (f32_t)(difEncoderCnt) * pIncABZ->eleSpeedCalculateFacotr * 0.1f + eleSpeed * 0.9f;

    return eleSpeed;

    // static uint32_t lastEncoderCnt = 0u;
    // uint32_t nowEncoderCnt = LL_TIM_GetCounter(TIM8);
    // int32_t difEncoderCnt;
    // if(pIncABZ->zeroPassABZCnt){
    //     if(LL_TIM_GetDirection(TIM8) == LL_TIM_COUNTERDIRECTION_UP){
    //         difEncoderCnt = (int32_t)(10000u + nowEncoderCnt - lastEncoderCnt);
    //     }
    //     else{
    //         difEncoderCnt = (int32_t)(nowEncoderCnt - lastEncoderCnt - 10000u);
    //     }
    //     pIncABZ->zeroPassABZCnt = 0u;
    // }
    // else{
    //     difEncoderCnt = nowEncoderCnt - lastEncoderCnt;
    // }
    // lastEncoderCnt = nowEncoderCnt;
    // eleSpeed = (f32_t)(difEncoderCnt) * 0.8f * 3.141592653589793f * 0.1f + eleSpeed * 0.9f;
    // return eleSpeed;
}

f32_t Hardware_GetRealEleAngle(void)
{
    int32_t realAngle = (LL_TIM_GetCounter(TIM8) % pIncABZ->encoderPPR_Uint);
    return (f32_t)realAngle / pIncABZ->eleAngleCalculateFacotr * MATH_PI - MATH_PI;
}

