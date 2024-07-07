/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-05-16 16:40:13
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-06-23 09:08:37
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc\FOC\source\mcFluxObs.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcFluxObs.h"
#include "../include/mcMath.h"

void NonlinearFluxPLLObs(volatile NonlinearFluxObsHandler* pNLFO,volatile PIC* sp,f32_t cos,f32_t sin)
{
    f32_t ts = pNLFO->ts;
    f32_t in;

    Components2 sinCosX = CalculateSinCosValue(pNLFO->integrator4);

    in = -cos * sinCosX.com1 + sin * sinCosX.com2;

    pNLFO->err = in;

    pNLFO->integrator3 += in * ts;
    f32_t speed = pNLFO->kP * in + pNLFO->integrator3 * pNLFO->kI;
    pNLFO->est_eleSpeed = speed;
    pNLFO->est_eleSpeedLPF = speed * 0.00001f + pNLFO->est_eleSpeedLPF * 0.99999f;
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
}

void NonlinearFluxObsProcess(volatile NonlinearFluxObsHandler* pNLFO,volatile PIC* sp,Components2* uAlphaBeta,Components2* iAlphaBeta)
{

    f32_t L = pNLFO->Ls;
    f32_t R = pNLFO->Rs;
    f32_t Flux = pNLFO->Flux;
    f32_t gamma = pNLFO->gamma;
    f32_t ts = pNLFO->ts;

    f32_t iAlpha = iAlphaBeta->com1;
    f32_t iBeta = iAlphaBeta->com2;

    f32_t uAlpha = uAlphaBeta->com1;
    f32_t uBeta = uAlphaBeta->com2;

    f32_t delay1 = pNLFO->delay1;
    f32_t delay2 = pNLFO->delay2;

    f32_t tmp0 = iAlpha * L;
    f32_t tmp1 = delay1 - tmp0;

    f32_t tmp2 = iBeta * L;
    f32_t tmp3 = delay2 - tmp2;

    f32_t eta = Flux * Flux - (tmp1 * tmp1 + tmp2 * tmp2);

    tmp1 = tmp1 * gamma * eta;
    tmp3 = tmp3 * gamma * eta;

    pNLFO->integrator1 += (tmp1 - iAlpha * R + uAlpha) * ts;
    pNLFO->integrator2 += (tmp3 - iBeta * R + uBeta) * ts;

    pNLFO->delay1 = pNLFO->integrator1;
    pNLFO->delay2 = pNLFO->integrator2;

    tmp0 = pNLFO->integrator1 - tmp0;
    tmp2 = pNLFO->integrator2 - tmp2;

    pNLFO->cos = tmp0 / Flux;
    pNLFO->sin = tmp2 / Flux;

    // f32_t x = FastReciprocalSquareRoot(pNLFO->cos * pNLFO->cos + pNLFO->sin * pNLFO->sin);

    // pNLFO->cos *= x;
    // pNLFO->sin *= x;

    NonlinearFluxPLLObs(pNLFO,sp,pNLFO->cos,pNLFO->sin);
}


