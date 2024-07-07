/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-12 13:28:22
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-07-07 12:24:35
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\include\mcMath.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _MC_MATH_H_
#define _MC_MATH_H_

#include "mcType.h"

#define MATH_PI 3.141592653589793f
#define abs(x) ((x) > 0.f) ? (x) : -(x)

extern CCMRAM Components2 Abc_AlphaBeta_Trans(volatile Components2* ab);
extern CCMRAM Components3 AlphaBeta_Abc_Trans(volatile Components2* alphaBeta);
extern CCMRAM Components2 AlphaBeta_Dq_Trans(volatile Components2* alphaBeta,volatile Components2* sinCos);
extern CCMRAM Components2 Dq_AlphaBeta_Trans(volatile Components2* dq,volatile Components2* sinCos);

extern CCMRAM Components2 CalculateSinCosValue(f32_t eleAngle);
extern CCMRAM f32_t FastSquareRoot(f32_t x);
extern CCMRAM f32_t FastReciprocalSquareRoot(f32_t x);
extern CCMRAM f32_t Min3(f32_t x1,f32_t x2,f32_t x3);
extern CCMRAM f32_t Max3(f32_t x1,f32_t x2,f32_t x3);

extern CCMRAM f32_t MedianFilter(f32_t datBuffer[],uint16_t length);
extern CCMRAM f32_t atan2Rad(f32_t x1,f32_t x2);

#endif

