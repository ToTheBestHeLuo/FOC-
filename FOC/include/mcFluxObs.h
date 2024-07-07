/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-05-16 16:39:25
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-19 22:56:16
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc\FOC\include\mcFluxObs.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _MC_FLUXOBS_H_
#define _MC_FLUXOBS_H_

#include "mcVar.h"

extern void NonlinearFluxObsProcess(volatile NonlinearFluxObsHandler* pNLFO,volatile PIC* sp,Components2* uAlphaBeta,Components2* iAlphaBeta);

#endif
