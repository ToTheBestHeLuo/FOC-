/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-23 18:44:48
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-06-20 16:19:41
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc\FOC\include\mcTask.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _MC_TASK_H_
#define _MC_TASK_H_

#include "mcVar.h"

extern void SafetyTask(void);
extern CCMRAM void PerformanceCriticalTask(void);
extern CCMRAM void SectorCalModeSvpwm(volatile SvpwmHandler* svp,f32_t busVoltage);
extern CCMRAM Components2 CurrentPIController(volatile PIC* idPIC,volatile PIC* iqPIC,volatile Components2* idqReal);

extern CCMRAM f32_t SpeedPIController(volatile PIC* speedPIC,f32_t realSpeed);

#endif



