/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-23 18:44:48
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-04-28 10:40:57
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431_mc_withABZ\FOC\include\mcHFI.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _MC_HFI_H_
#define _MC_HFI_H_

#include "mcVar.h"

extern f32_t HFPISensorlessObserver(volatile SensorHandler* sens,volatile HFPIHandler* hfpi);
extern f32_t HFSISensorlessObserver(volatile SensorHandler* sens,volatile HFSIHandler* hfsi);
extern f32_t HFSI_AngleCompensate(volatile HFSIHandler* hfsi);
extern bool NSIdentifyStateMachine(volatile NSIdentifyProcessHandler* ns,f32_t currentId);
extern bool NSCheckStateMachine(volatile NSCheckHandler* ns);

#endif


