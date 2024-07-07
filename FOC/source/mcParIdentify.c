/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-04-11 10:06:36
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-31 12:25:31
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc\FOC\source\mcParIdentify.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcParIdentify.h"
#include "../include/mcMath.h"

f32_t BPF_Order_2st(f32_t in0)
{
    static f32_t out1 = 0.f;
    static f32_t out2 = 0.f;
    static f32_t in1 = 0.f;

    f32_t out0 = 1.954f * out1 - 0.9691f * out2 + 0.03085f * in0 - 0.03085f * in1;

    out2 = out1;
    out1 = out0;

    in1 = in0;

    return out2;
}

Components2 LPF_Order_2st(volatile Components2* in0)
{
    static Components2 out2 = {0.f,0.f};
    static Components2 out1 = {0.f,0.f};
    static Components2 out0 = {0.f,0.f};
    static Components2 in1 = {0.f,0.f};

    out2.com1 = 1.956f * out1.com1 - 0.9565f * out0.com1 + 0.0004862f * in1.com1 + 0.0004791f * in0->com1;
    in1.com1 = in0->com1;
    out1.com1 = out2.com1;
    out0.com1 = out1.com1;

    out2.com2 = 1.956f * out1.com2 - 0.9565f * out0.com2 + 0.0004862f * in1.com2 + 0.0004791f * in0->com2;
    in1.com2 = in0->com2;
    out1.com2 = out2.com2;
    out0.com2 = out1.com2;

    return out2; 
}

Components2 LPF_RMS(volatile Components2* in)
{
    Components2 rms;
    static Components2 totalSquare = {0.f,0.f};
    static f32_t nums = 0.f;
    totalSquare.com1 += in->com1 * in->com1;
    totalSquare.com2 += in->com2 * in->com2;
    rms.com1 = FastSquareRoot(totalSquare.com1 / (nums + 1.f));
    rms.com2 = FastSquareRoot(totalSquare.com2 / (nums + 1.f));
    nums += 1.f;
    return rms;
}

void MCParIdentify_Rs_Ls(volatile MC_ParameterIdentify_Handler* parHandler,f32_t id)
{
    static uint32_t timeCnt = 0u;

    f32_t idHF = BPF_Order_2st(id);

    parHandler->demodulation_Phase = 2.f * MATH_PI * parHandler->injectFre * (f32_t)timeCnt * parHandler->ts + parHandler->demodulation_phaseCompensate;

    Components2 sinCos = CalculateSinCosValue(parHandler->demodulation_Phase);
    Components2 demodulation_sinCos = {parHandler->demodulation_ampCompensate * sinCos.com1,parHandler->demodulation_ampCompensate * sinCos.com2};
    parHandler->demodulation_sinCos = demodulation_sinCos;
    Components2 sigHF = {idHF * demodulation_sinCos.com1,idHF * demodulation_sinCos.com2};
    parHandler->sig_HF = sigHF;

    parHandler->sig_LF = LPF_Order_2st(&parHandler->sig_HF);
    parHandler->sig = LPF_RMS(&parHandler->sig_LF);

    f32_t z = 2.f * FastSquareRoot(parHandler->sig.com1 * parHandler->sig.com1 + parHandler->sig.com2 * parHandler->sig.com2);
    f32_t phase = atan2Rad(parHandler->sig.com2,parHandler->sig.com1);
		
	z = parHandler->injectSigAmp * 1.273f / z * 1.628177966101695f;
		
    if(z > 10.f){
        z = 0.f;
    }

    phase = -phase;

    sinCos = CalculateSinCosValue(phase);
    parHandler->mc_Rs = z * sinCos.com2;
    parHandler->mc_Ls = -z * sinCos.com1 / 2.f / MATH_PI / parHandler->injectFre;

    timeCnt++;
}


