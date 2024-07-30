/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-07-17 14:40:07
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-07-26 14:09:59
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\source\mcSensor.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcSensor.h"
#include "../include/mcVar.h"
#include "../interface/mcConfig.h"
#include "../include/mcMath.h"

f32_t IncAbzCalculateRealEleAngle(volatile IncABZEncoder* pABZ)
{
    int32_t realAngle = (Hardware_GetABZCounter() % pABZ->encoderPPR_Uint);
    return (f32_t)realAngle / pABZ->eleAngleCalculateFacotr * MATH_PI - MATH_PI;
}

f32_t IncAbzCalculateRealEleSpeed(volatile IncABZEncoder* pABZ,f32_t targetEleSpeed)
{
    static f32_t eleSpeed = 0.f;
    static uint8_t speedSta = 0xFF;
    static f32_t calFactor = 0.f;
    static int32_t difEncoderCnt = 0;
    
    uint32_t pulseCnt;

    int32_t lastEncoderCnt = pABZ->lastEncoderCnt;
    int32_t nowEncoderCnt = Hardware_GetABZCounter();

    pulseCnt = Hardware_GetPulseCounter();
    Hardware_SetPulseCounter(0);

    int32_t diff = nowEncoderCnt - lastEncoderCnt;
    if(diff > -incABZHandler.encoderPPR_XX_Uint / 4 && diff < incABZHandler.encoderPPR_XX_Uint / 4){
        difEncoderCnt = diff;
    }

    pIncABZ->lastEncoderCnt = nowEncoderCnt;

    if(targetEleSpeed < 0.f) targetEleSpeed = -targetEleSpeed;

    if(speedSta){
        f32_t factor = 2.f * MATH_PI * (f32_t)pMotor->polePairs / ((f32_t)incABZHandler.encoderPPR_XX_Uint * (f32_t)pulseCnt * pSys->pulseSpeedClock);
        calFactor = factor * 0.1f + calFactor * 0.9f;
        eleSpeed = (f32_t)(difEncoderCnt) * calFactor * 0.1f + eleSpeed * 0.9f;
        if(targetEleSpeed > pIncABZ->highEleSpeedThreshold) speedSta = ~speedSta;
    }
    else{
        eleSpeed = (f32_t)(difEncoderCnt) * pABZ->eleSpeedCalculateFacotr * 0.1f + eleSpeed * 0.9f;
        if(targetEleSpeed < pIncABZ->lowEleSpeedThreshold) speedSta = ~speedSta;
    }

    return eleSpeed;
}






