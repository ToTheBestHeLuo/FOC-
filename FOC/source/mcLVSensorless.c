/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-07-17 11:08:21
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-08-01 16:35:01
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\source\mcLVSensorless.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcLVSensorless.h"
#include "../include/mcMath.h"
#include "../interface/mcConfig.h"
#include "../include/mcDigitalFilter.h"

static LPF_2stOrder_1in_1out_Handler lpf2stOrder0,lpf2stOrder1,lpf2stOrder2;
static BPF_2stOrder_1in_1out_Handler bpf2stOrder0,bpf2stOrder1;

void HFPI_LPF_2stOrder_1in_1out_SetPar(f32_t k1,f32_t k2,f32_t k3,f32_t k4)
{
    LPF_2stOrder_1in_1out_SetPar(&lpf2stOrder0,k1,k2,k3,k4);
    LPF_2stOrder_1in_1out_SetPar(&lpf2stOrder1,k1,k2,k3,k4);
    LPF_2stOrder_1in_1out_SetPar(&lpf2stOrder2,k1,k2,k3,k4);
}
void HFPI_BPF_2stOrder_1in_1out_SetPar(f32_t k1,f32_t k2,f32_t k3,f32_t k4)
{
    BPF_2stOrder_1in_1out_SetPar(&bpf2stOrder0,k1,k2,k3,k4);
    BPF_2stOrder_1in_1out_SetPar(&bpf2stOrder1,k1,k2,k3,k4);
}

f32_t HFPISensorlessObserver(volatile SensorHandler* sens,volatile HFPIHandler* hfpi)
{
    f32_t clk = pSys->highSpeedClock;

    hfpi->inject_phaseSinCos = Hardware_GetSinCosVal(hfpi->inject_phase);

    hfpi->inject_phase += 2.f * MATH_PI * hfpi->injectFrequency * clk;

    if(hfpi->inject_phase > MATH_PI){
        hfpi->inject_phase = hfpi->inject_phase - 2.f * MATH_PI;
    }
    else if(hfpi->inject_phase < -MATH_PI){
        hfpi->inject_phase = hfpi->inject_phase + 2.f * MATH_PI;
    }

    hfpi->response_iAlphaBeta = Abc_AlphaBeta_Trans(&sens->currentAB);
    hfpi->response_iDQ = AlphaBeta_Dq_Trans(&hfpi->response_iAlphaBeta,&sens->sinCosVal);

    hfpi->response_HF_iDQ.com1 = BPF_2stOrder_1in_1out_Calculate(&bpf2stOrder0,hfpi->response_iDQ.com1);
    hfpi->response_HF_iDQ.com2 = BPF_2stOrder_1in_1out_Calculate(&bpf2stOrder1,hfpi->response_iDQ.com2);

    sens->currentDQ.com1 = LPF_2stOrder_1in_1out_Calculate(&lpf2stOrder0,hfpi->response_iDQ.com1 - hfpi->response_HF_iDQ.com1);
    sens->currentDQ.com1 = LPF_2stOrder_1in_1out_Calculate(&lpf2stOrder1,hfpi->response_iDQ.com2 - hfpi->response_HF_iDQ.com2);

    f32_t tmp = hfpi->response_HF_iDQ.com2 * hfpi->inject_phaseSinCos.com1;

    tmp = LPF_2stOrder_1in_1out_Calculate(&lpf2stOrder2,tmp);

    hfpi->est_err = tmp;

    static f32_t errInt0 = 0.f;
    static f32_t errInt1 = 0.f;

    errInt0 += hfpi->est_err * clk;

    f32_t tmp0 = errInt0 * hfpi->PLL_Ki + hfpi->est_err * hfpi->PLL_Kp;

    hfpi->est_eleSpeed = tmp0;

    errInt1 += tmp0 * clk;

    if(errInt1 > MATH_PI)
        errInt1 = -MATH_PI * 2.f + errInt1;
    else if(errInt1 < -MATH_PI)
        errInt1 = MATH_PI * 2.f - errInt1;

    hfpi->est_eleAngle = errInt1;

    return hfpi->inject_phaseSinCos.com2 * hfpi->injectVoltage;
}

void HFSI_Observer(volatile HFSIHandler* hfsi)
{
    f32_t err = hfsi->est_err;

    f32_t ts = pSys->highSpeedClock;

    hfsi->int1 += ts * err;

    f32_t eleSpeed = hfsi->int1 * hfsi->kI + err * hfsi->kP;

    hfsi->est_eleSpeed = eleSpeed * 0.005f + hfsi->est_eleSpeed * 0.995f;

    f32_t tmp = hfsi->int2 + ts * eleSpeed;

    if(tmp > MATH_PI){
        hfsi->int2 = -MATH_PI * 2.f + tmp;
    }
    else if(tmp < -MATH_PI){
        hfsi->int2 = MATH_PI * 2.f + tmp;
    }
    else{
        hfsi->int2 = tmp;
    }

    hfsi->est_eleAngle = hfsi->int2;
}

f32_t HFSISensorlessObserver(volatile SensorHandler* sens,volatile HFSIHandler* hfsi)
{
    hfsi->response_iAlphaBeta = Abc_AlphaBeta_Trans(&sens->currentAB);

    Components2 iDQNow = AlphaBeta_Dq_Trans(&hfsi->response_iAlphaBeta,&sens->sinCosVal);

    sens->currentDQ = iDQNow;
    hfsi->response_iDQ = iDQNow;

    hfsi->response_LF_iDQ.com1 = 0.5f * (iDQNow.com1 + hfsi->iDQLast.com1);
    hfsi->response_LF_iDQ.com2 = 0.5f * (iDQNow.com2 + hfsi->iDQLast.com2);

    f32_t iAlphaHF = (hfsi->response_iAlphaBeta.com1 - hfsi->iAlphaBetaLast.com1) * 0.5f;
    f32_t iBetaHF = (hfsi->response_iAlphaBeta.com2 - hfsi->iAlphaBetaLast.com2) * 0.5f;

    f32_t iAlphaLF = (hfsi->response_iAlphaBeta.com1 + hfsi->iAlphaBetaLast.com1) * 0.5f;
    f32_t iBetaLF = (hfsi->response_iAlphaBeta.com2 + hfsi->iAlphaBetaLast.com2) * 0.5f;

    hfsi->response_LF_iAlphaBeta.com1 = iAlphaLF;
    hfsi->response_LF_iAlphaBeta.com2 = iBetaLF;

    if(hfsi->inject_polarity){
        iAlphaHF *= -1.f;
        iBetaHF *= -1.f;
    }

    hfsi->response_HF_iAlphaBeta.com1 = iAlphaHF;
    hfsi->response_HF_iAlphaBeta.com2 = iBetaHF;

    f32_t sqrt = FastReciprocalSquareRoot(hfsi->response_HF_iAlphaBeta.com1 * hfsi->response_HF_iAlphaBeta.com1 + hfsi->response_HF_iAlphaBeta.com2 * hfsi->response_HF_iAlphaBeta.com2);
    hfsi->response_HF_iAlphaBetaPerUnit.com1 = hfsi->response_HF_iAlphaBeta.com1 * sqrt;
    hfsi->response_HF_iAlphaBetaPerUnit.com2 = hfsi->response_HF_iAlphaBeta.com2 * sqrt;

    hfsi->est_err = -hfsi->response_HF_iAlphaBetaPerUnit.com1 * sens->sinCosVal.com1 + hfsi->response_HF_iAlphaBetaPerUnit.com2 * sens->sinCosVal.com2;

    HFSI_Observer(hfsi);

    hfsi->iAlphaBetaLast = hfsi->response_iAlphaBeta;
    hfsi->iDQLast = iDQNow;
    
    hfsi->inject_polarity = !hfsi->inject_polarity;

    hfsi->inject_voltage = -hfsi->inject_voltage;

    return hfsi->inject_voltage;
}

bool NSIdentifyStateMachine(volatile NSIdentifyProcessHandler* ns,f32_t currentId)
{
    pNSIdentify->isCompensateFinished = false;

    if(ns->maxId < currentId)
        ns->maxId = currentId;
    if(ns->minId > currentId)
        ns->minId = currentId;

    if(ns->pulseWidthCnt++ < 1 * ns->pulseWidth){
        ns->inject_voltage = ns->inject_amp;
    }
    else if(ns->pulseWidthCnt < 2 * ns->pulseWidth){
        ns->inject_voltage = 0.f;
    }
    else if(ns->pulseWidthCnt < 3 * ns->pulseWidth){
        ns->inject_voltage = -ns->inject_amp;
    }
    else if(ns->pulseWidthCnt < 4 * ns->pulseWidth){
        ns->inject_voltage = 0.f;
    }
    else{
        if(ns->maxId + ns->minId > ns->posGate){
            if(ns->polarityCnt++ > ns->polarityCntGate){
                pNSIdentify->isCompensateFinished = true;
                ns->nsCompensate = 0.f;
            }
        }
        else if(ns->maxId + ns->minId < ns->negGate){
            if(ns->polarityCnt-- < -ns->polarityCntGate){
                pNSIdentify->isCompensateFinished = true;
                ns->nsCompensate = MATH_PI;
            }
        }
        ns->pulseWidthCnt = 0;
        ns->maxId = ns->minId = 0.f;
    }

    return pNSIdentify->isCompensateFinished;
}



