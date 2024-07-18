/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-07-17 14:40:07
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-07-18 10:57:26
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\source\mcSensor.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcSensor.h"
#include "../include/mcVar.h"
#include "../interface/mcConfig.h"
#include "../include/mcMath.h"

f32_t IncAbzCalculateRealEleAngle(void)
{
    int32_t realAngle = (Hardware_GetABZCounter() % pIncABZ->encoderPPR_Uint);
    return (f32_t)realAngle / pIncABZ->eleAngleCalculateFacotr * MATH_PI - MATH_PI;
}

f32_t IncAbzCalculateRealEleSpeed(void)
{
    static f32_t realEleSpeed = 0.f;

    uint32_t lastEncoderCnt = pIncABZ->lastEncoderCnt;
    uint32_t nowEncoderCnt = Hardware_GetABZCounter();
    int32_t difEncoderCnt = 0;

    uint32_t dir = Hardware_GetABZCounterDir();

    if(dir && pIncABZ->dirLPF++ == 10){
        pIncABZ->motorRunSta = 1;
        pIncABZ->dirLPF = 0;
    }
    else if(!dir && pIncABZ->dirLPF-- == -10){
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
    realEleSpeed = (f32_t)(difEncoderCnt) * pIncABZ->eleSpeedCalculateFacotr * 0.1f + realEleSpeed * 0.9f;

    return realEleSpeed;
}






