/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-16 11:32:49
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-06-16 10:01:41
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc\FOC\source\mcHFI.c
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcHFI.h"
#include "../include/mcMath.h"
#include "../interface/mcConfig.h"

Components2 HFPI_BPF1_2st(volatile Components2* in0)
{
    static Components2 out1 = {0.f,0.f};
    static Components2 out2 = {0.f,0.f};
    static Components2 in1 = {0.f,0.f};

    f32_t out0 = 1.907f * out1.com1 - 0.9691f * out2.com1 + 0.0306f * in0->com1 - 0.0306f * in1.com1;

    out2.com1 = out1.com1;
    out1.com1 = out0;

    in1.com1 = in0->com1;

    out0 = 1.907f * out1.com2 - 0.9691f * out2.com2 + 0.0306f * in0->com2 - 0.0306f * in1.com2;

    out2.com2 = out1.com2;
    out1.com2 = out0;

    in1.com2 = in0->com2;

    return out2; 
}

Components2 HFPI_LPF1_2st(volatile Components2* in0)
{
    static Components2 out1 = {0.f,0.f};
    static Components2 out2 = {0.f,0.f};
    static Components2 in1 = {0.f,0.f};

    f32_t out0 = 1.911f * out1.com1 - 0.915f * out2.com1 + 0.001916f * in0->com1 + 0.00186f * in1.com1;

    out2.com1 = out1.com1;
    out1.com1 = out0;

    in1.com1 = in0->com1;

    out0 = 1.911f * out1.com2 - 0.915f * out2.com2 + 0.001916f * in0->com2 + 0.00186f * in1.com2;

    out2.com2 = out1.com2;
    out1.com2 = out0;

    in1.com2 = in0->com2;

    return out2; 
}

Components2 HFPI_LPF2_2st(Components2* in0)
{
    static Components2 out1 = {0.f,0.f};
    static Components2 out2 = {0.f,0.f};
    static Components2 in1 = {0.f,0.f};

    f32_t out0 = 1.911f * out1.com1 - 0.915f * out2.com1 + 0.001916f * in0->com1 + 0.00186f * in1.com1;

    out2.com1 = out1.com1;
    out1.com1 = out0;

    in1.com1 = in0->com1;

    out0 = 1.911f * out1.com2 - 0.915f * out2.com2 + 0.001916f * in0->com2 + 0.00186f * in1.com2;

    out2.com2 = out1.com2;
    out1.com2 = out0;

    in1.com2 = in0->com2;

    return out2; 
}

f32_t HFPISensorlessObserver(volatile SensorHandler* sens,volatile HFPIHandler* hfpi)
{
    static uint32_t timeCnt = 0u;
    hfpi->inject_phaseSinCos = Hardware_GetSinCosVal(hfpi->inject_phase);
    hfpi->inject_phase = ((f32_t)(timeCnt++) * PerformanceCriticalTask_Period) * 2.f * MATH_PI * 400.f;
    hfpi->response_iAlphaBeta = Abc_AlphaBeta_Trans(&sens->currentAB);
    hfpi->response_iDQ = AlphaBeta_Dq_Trans(&hfpi->response_iAlphaBeta,&sens->sinCosVal);
    sens->currentDQ = hfpi->response_iDQ;

    hfpi->response_HF_iDQ = HFPI_BPF1_2st(&hfpi->response_iDQ);

    hfpi->response_LF_iDQ = HFPI_LPF1_2st(&hfpi->response_iDQ);

    Components2 tmp = {-hfpi->response_HF_iDQ.com1 * hfpi->inject_phaseSinCos.com1,hfpi->response_HF_iDQ.com2 * hfpi->inject_phaseSinCos.com1};

    tmp = HFPI_LPF2_2st(&tmp);

    hfpi->est_err = tmp.com2;

    static f32_t errInt0 = 0.f;
    static f32_t errInt1 = 0.f;

    errInt0 += hfpi->est_err * PerformanceCriticalTask_Period;
    f32_t tmp0 = errInt0 * 4.f + hfpi->est_err * 200.f;

    hfpi->est_eleSpeed = tmp0 * 0.001f + hfpi->est_eleSpeed * 0.999f;

    errInt1 += tmp0 * PerformanceCriticalTask_Period;

    if(errInt1 > MATH_PI)
        errInt1 = -MATH_PI * 2.f + errInt1;
    else if(errInt1 < -MATH_PI)
        errInt1 = MATH_PI * 2.f - errInt1;

    hfpi->est_eleAngle = errInt1;

    return hfpi->inject_phaseSinCos.com2 * 1.f;
}

void HFSI_Observer(volatile HFSIHandler* hfsi)
{
    f32_t err = hfsi->est_err;

    f32_t ts = hfsi->ts;

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

f32_t HFSI_AngleCompensate(volatile HFSIHandler* hfsi)
{
    f32_t x1 = hfsi->response_HF_iDQ.com1;
    f32_t x2 = hfsi->response_HF_iDQ.com2;

    f32_t root = hfsi->response_LF_iDQ.com1 * hfsi->response_LF_iDQ.com1 + hfsi->response_LF_iDQ.com2 * hfsi->response_LF_iDQ.com2;
    root = Hardware_FastSquareRoot(root);

    f32_t sign1 = (x1 * x2 > 0.f) ? 1.f : -1.f;
    f32_t sign2 = (hfsi->response_LF_iDQ.com2 > 0.f) ? 1.f : -1.f;

    f32_t compensate = sign1 * atan2Rad(x2,x1) + sign2 * atan2Rad(0.00005f*root,0.0077f);

    return compensate;
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

bool NSCheckStateMachine(volatile NSCheckHandler* ns)
{
    uint8_t sta = ns->status;

    if(ns->maxId < ns->iDQ.com1)
        ns->maxId = ns->iDQ.com1;
    if(ns->minId > ns->iDQ.com1)
        ns->minId = ns->iDQ.com1; 

    if(sta == 0){
        ns->injectVoltage = ns->injectPosVoltage;
        if(ns->pulseWidthCnt++ > ns->pulseWidth){
            ns->pulseWidthCnt = 0;
            ns->status = 1;
        }
    }
    else if(sta == 1){
        ns->injectVoltage = ns->injectZeroVoltage;
        if(ns->pulseWidthCnt++ > ns->pulseWidth){
            ns->pulseWidthCnt = 0;
            ns->status = 2;
        }
    }
    else if(sta  == 2){
        ns->injectVoltage = ns->injectNegVoltage;
        if(ns->pulseWidthCnt++ > ns->pulseWidth){
            ns->pulseWidthCnt = 0;
            ns->status = 3;
        }
    }
    else if(sta == 3){
        ns->injectVoltage = ns->injectZeroVoltage;
        if(ns->pulseWidthCnt++ > ns->pulseWidth){
            ns->pulseWidthCnt = 0;
            ns->status = 4;
        }
    }
    else{
        if(ns->minId + ns->maxId > 0.f){
            if(ns->polarityCnt++ > 5){
                ns->compensateAngle = 0.f;
                ns->isFinished = true;
            }
        }
        else if(ns->minId + ns->maxId < 0.f){
            if(ns->polarityCnt-- < -5){
                ns->compensateAngle = MATH_PI;
                ns->isFinished = true;
            }          
        }
        ns->minId = 0.f;
        ns->maxId = 0.f;
        ns->status = 0;
    }

    return ns->isFinished;
}

