/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-08-01 11:05:36
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-08-01 13:15:56
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\include\mcDigitalFilter.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _MC_DIGITALFILTER_H_
#define _MC_DIGITALFILTER_H_

#include "mcType.h"

typedef struct
{
    f32_t k1,k2;
    f32_t out1;
}LPF_1stOrder_1in_1out_Handler;

typedef struct 
{
    f32_t k1,k2,k3,k4;
    f32_t out1,out2,in1;
}LPF_2stOrder_1in_1out_Handler;

typedef struct 
{
    f32_t k1,k2,k3,k4;
    f32_t out1,out2,in1;
}BPF_2stOrder_1in_1out_Handler;

extern void LPF_1stOrder_1in_1out_SetPar(LPF_1stOrder_1in_1out_Handler* lpf,f32_t k1,f32_t k2);
extern f32_t LPF_1stOrder_1in_1out_Calculate(LPF_1stOrder_1in_1out_Handler* lpf,f32_t in);
extern void LPF_2stOrder_1in_1out_SetPar(LPF_2stOrder_1in_1out_Handler* lpf,f32_t k1,f32_t k2,f32_t k3,f32_t k4);
extern f32_t LPF_2stOrder_1in_1out_Calculate(LPF_2stOrder_1in_1out_Handler* lpf,f32_t in);
extern void BPF_2stOrder_1in_1out_SetPar(BPF_2stOrder_1in_1out_Handler* bpf,f32_t k1,f32_t k2,f32_t k3,f32_t k4);
extern f32_t BPF_2stOrder_1in_1out_Calculate(BPF_2stOrder_1in_1out_Handler* bpf,f32_t in);

#endif

