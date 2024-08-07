/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-08-01 11:07:24
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-08-07 11:07:09
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\source\mcDigitalFilter.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcDigitalFilter.h"
#include "../include/mcVar.h"
/**
 * @brief: 配置一阶RC数字低通滤波器
 * @param {LPF_1stOrder_1in_1out_Handler*} lpf
 * @param {f32_t} k1
 * @param {f32_t} k2
 * @return {*}
 * @note: none
 * @description: 
 */
void LPF_1stOrder_1in_1out_SetPar(LPF_1stOrder_1in_1out_Handler* lpf,f32_t k1,f32_t k2)
{
    lpf->k1 = k1;
    lpf->k2 = k2;
    lpf->out1 = 0.f;
}
/**
 * @brief: 计算一阶RC数字低通滤波器的输出
 * @param {LPF_1stOrder_1in_1out_Handler*} lpf
 * @param {f32_t} in
 * @return {f32_t}
 * @note: none
 * @description: 
 */
f32_t LPF_1stOrder_1in_1out_Calculate(LPF_1stOrder_1in_1out_Handler* lpf,f32_t in)
{
    lpf->out1 = lpf->out1 * lpf->k1 + in * lpf->k2;
    return lpf->out1;
}
/**
 * @brief: 配置二阶数字低通滤波器
 * @param {LPF_2stOrder_1in_1out_Handler*} lpf
 * @param {f32_t} k1
 * @param {f32_t} k2
 * @param {f32_t} k3
 * @param {f32_t} k4
 * @return {*}
 * @note: none
 * @description: 
 */
void LPF_2stOrder_1in_1out_SetPar(LPF_2stOrder_1in_1out_Handler* lpf,f32_t k1,f32_t k2,f32_t k3,f32_t k4)
{
    lpf->k1 = k1;
    lpf->k2 = k2;
    lpf->k3 = k3;
    lpf->k4 = k4;
    lpf->out1 = lpf->out2 = lpf->in1 = 0.f;
}
/**
 * @brief: 计算二阶数字低通滤波器的输出
 * @param {LPF_2stOrder_1in_1out_Handler*} lpf
 * @param {f32_t} in
 * @return {f32_t}
 * @note: none
 * @description: 
 */
f32_t LPF_2stOrder_1in_1out_Calculate(LPF_2stOrder_1in_1out_Handler* lpf,f32_t in)
{
    f32_t out0 = lpf->k1 * lpf->out1 + lpf->k2 * lpf->out2 + lpf->k3 * in + lpf->k4 * lpf->in1;
    lpf->out2 = lpf->out1;
    lpf->out1 = out0;
    lpf->in1 = in;
    return out0;
}
/**
 * @brief: 配置二阶数字带通滤波器
 * @param {LPF_2stOrder_1in_1out_Handler*} lpf
 * @param {f32_t} k1
 * @param {f32_t} k2
 * @param {f32_t} k3
 * @param {f32_t} k4
 * @return {*}
 * @note: none
 * @description: 
 */
void BPF_2stOrder_1in_1out_SetPar(BPF_2stOrder_1in_1out_Handler* bpf,f32_t k1,f32_t k2,f32_t k3,f32_t k4)
{
    bpf->k1 = k1;
    bpf->k2 = k2;
    bpf->k3 = k3;
    bpf->k4 = k4;
    bpf->out1 = bpf->out2 = bpf->in1 = 0.f;
}
/**
 * @brief: 计算二阶数字带通滤波器的输出
 * @param {LPF_2stOrder_1in_1out_Handler*} lpf
 * @param {f32_t} in
 * @return {f32_t}
 * @note: none
 * @description: 
 */
f32_t BPF_2stOrder_1in_1out_Calculate(BPF_2stOrder_1in_1out_Handler* bpf,f32_t in)
{
    f32_t out0 = bpf->k1 * bpf->out1 + bpf->k2 * bpf->out2 + bpf->k3 * in + bpf->k4 * bpf->in1;
    bpf->out2 = bpf->out1;
    bpf->out1 = out0;
    bpf->in1 = in;
    return out0;
}


