/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-07-17 14:40:07
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-07-20 11:10:54
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
    static f32_t realEleSpeed = 0.f;
    static uint8_t speedSta = 0xFF;
    
    uint32_t pulseCnt;

    uint32_t lastEncoderCnt = pABZ->lastEncoderCnt;
    uint32_t nowEncoderCnt = Hardware_GetABZCounter();

    pulseCnt = Hardware_GetPulseCounter();
    uint32_t dir = Hardware_GetABZCounterDir();
    Hardware_SetPulseCounter(0);

    int32_t difEncoderCnt = 0;

    if(dir && pABZ->dirLPF++ == 4){
        pABZ->motorRunSta = 1;
        pABZ->dirLPF = 0;
    }
    else if(!dir && pABZ->dirLPF-- == -4){
        pABZ->motorRunSta = 0;
        pABZ->dirLPF = 0;
    }

    if(pABZ->zeroPassABZCnt){
        if(pABZ->motorRunSta == 1){
            difEncoderCnt = (int32_t)(incABZHandler.encoderPPR_XX_Uint + nowEncoderCnt - lastEncoderCnt);
        }
        else if(pABZ->motorRunSta == 0){
            difEncoderCnt = (int32_t)(nowEncoderCnt - lastEncoderCnt - incABZHandler.encoderPPR_XX_Uint);
        }
        pABZ->zeroPassABZCnt = 0u;
    }
    else{
        difEncoderCnt = nowEncoderCnt - lastEncoderCnt;
    }

    pIncABZ->lastEncoderCnt = nowEncoderCnt;

    if(targetEleSpeed < 0.f) targetEleSpeed = -targetEleSpeed;

    if(speedSta){
        f32_t calFactor = 2.f * MATH_PI * (f32_t)pMotor->polePairs / ((f32_t)incABZHandler.encoderPPR_XX_Uint * (f32_t)pulseCnt * pSys->pulseSpeedClock);
        realEleSpeed = (f32_t)(difEncoderCnt) * calFactor * 0.1f + realEleSpeed * 0.9f;
        if(targetEleSpeed > pIncABZ->highEleSpeedThreshold) speedSta = ~speedSta;
    }
    else{
        realEleSpeed = (f32_t)(difEncoderCnt) * pABZ->eleSpeedCalculateFacotr * 0.1f + realEleSpeed * 0.9f;

        if(targetEleSpeed < pIncABZ->lowEleSpeedThreshold) speedSta = ~speedSta;
    }


    return realEleSpeed;
}






