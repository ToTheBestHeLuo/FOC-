/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-07-17 11:07:25
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-08-01 13:24:39
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\include\mcLVSensorless.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _MC_LVSENSORLESS_H_
#define _MC_LVSENSORLESS_H_

#include "mcVar.h"

extern void HFPI_LPF_2stOrder_1in_1out_SetPar(f32_t k1,f32_t k2,f32_t k3,f32_t k4);
extern void HFPI_BPF0_2stOrder_1in_1out_SetPar(f32_t k1,f32_t k2,f32_t k3,f32_t k4);
extern void HFPI_BPF1_2stOrder_1in_1out_SetPar(f32_t k1,f32_t k2,f32_t k3,f32_t k4);
extern f32_t HFPISensorlessObserver(volatile SensorHandler* sens,volatile HFPIHandler* hfpi);

extern f32_t HFSISensorlessObserver(volatile SensorHandler* sens,volatile HFSIHandler* hfsi);
extern bool NSIdentifyStateMachine(volatile NSIdentifyProcessHandler* ns,f32_t currentId);

#endif
