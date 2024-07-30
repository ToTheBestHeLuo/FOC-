/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-07-17 10:54:54
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-07-30 16:07:29
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\source\mcHVSensorless.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcHVSensorless.h"
#include "../include/mcMath.h"

void NonlinearFluxPLLObs(volatile NonlinearFluxObsHandler* pNLFO,volatile PIC* sp,f32_t cos,f32_t sin)
{
    f32_t ts = pSys->highSpeedClock;
    f32_t in;

    Components2 sinCosX = CalculateSinCosValue(pNLFO->integrator4);
    in = -cos * sinCosX.com1 + sin * sinCosX.com2;
    pNLFO->err = in;

    pNLFO->integrator3 += in * ts;
    f32_t speed = pNLFO->kP * in + pNLFO->integrator3 * pNLFO->kI;
    pNLFO->est_eleSpeed = speed;
    pNLFO->integrator4 += pNLFO->est_eleSpeed * ts;
    if(pNLFO->integrator4 > MATH_PI){
        pNLFO->integrator4 = -MATH_PI * 2.f + pNLFO->integrator4;
    }
    else if(pNLFO->integrator4 < -MATH_PI){
        pNLFO->integrator4 = MATH_PI * 2.f + pNLFO->integrator4;
    }
    else{
        pNLFO->integrator4 = pNLFO->integrator4;
    }
    pNLFO->est_eleAngle = pNLFO->integrator4;

    // f32_t ts = pSys->highSpeedClock;
    // f32_t estAngle = atan2Rad(sin,cos);
    // f32_t err = estAngle - pNLFO->integrator4;
    // pNLFO->integrator3 += err * ts;
    // pNLFO->est_eleSpeed = pNLFO->kP * err + pNLFO->integrator3 * pNLFO->kI;
    // pNLFO->integrator4 +=  pNLFO->est_eleSpeed * ts;
    // if(pNLFO->integrator4 > MATH_PI){
    //     pNLFO->integrator4 = -MATH_PI * 2.f + pNLFO->integrator4;
    // }
    // else if(pNLFO->integrator4 < -MATH_PI){
    //     pNLFO->integrator4 = MATH_PI * 2.f + pNLFO->integrator4;
    // }
    // else{
    //     pNLFO->integrator4 = pNLFO->integrator4;
    // }
    // pNLFO->est_eleAngle = estAngle;
}

void NonlinearFluxObsProcess(volatile MC_MotorPar* motor,volatile NonlinearFluxObsHandler* pNLFO,volatile PIC* sp,Components2* uAlphaBeta,Components2* iAlphaBeta)
{
    f32_t L = pMotor->Ls;
    f32_t R = pMotor->Rs;
    f32_t Flux = pMotor->Flux;
    f32_t gamma = pNLFO->gamma;
    f32_t ts = pSys->highSpeedClock;

    f32_t iAlpha = iAlphaBeta->com1;
    f32_t iBeta = iAlphaBeta->com2;

    f32_t uAlpha = uAlphaBeta->com1;
    f32_t uBeta = uAlphaBeta->com2;

    f32_t tmp0 = iAlpha * L;
    f32_t tmp1 = pNLFO->integrator1 - tmp0;

    f32_t tmp2 = iBeta * L;
    f32_t tmp3 = pNLFO->integrator2 - tmp2;

    f32_t eta = Flux * Flux - (tmp1 * tmp1 + tmp3 * tmp3);
 
    tmp1 = tmp1 * gamma * eta;
    tmp3 = tmp3 * gamma * eta;

    tmp0 = pNLFO->integrator1 - tmp0;
    tmp2 = pNLFO->integrator2 - tmp2;

    pNLFO->cos = tmp0 / Flux;
    pNLFO->sin = tmp2 / Flux;

    NonlinearFluxPLLObs(pNLFO,sp,pNLFO->cos,pNLFO->sin);

    pNLFO->integrator1 += (tmp1 - iAlpha * R + uAlpha) * ts;
    pNLFO->integrator2 += (tmp3 - iBeta * R + uBeta) * ts;
}


void LuenbergerObs()
{

}


