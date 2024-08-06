/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-23 18:44:48
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-08-01 16:33:21
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\interface\mcConfig.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _MC_CONFIG_H_
#define _MC_CONFIG_H_

#include "../include/mcType.h"
#include "../include/mcVar.h"

#define MC_SafeVoltage 36.f

#define Motor_PolePairs 4u
#define Motor_Ld (0.001f)
#define Motor_Lq (0.001f)
#define Motor_Ls (0.001f)
#define Motor_Rs (0.375f)
#define Motor_J (0.000001f)
#define Motor_Flux (0.011866667f)

#define ABZ_PPR 2500u

#define Timer_Period_ARR    8499
#define SafetyTask_Period   0.001f
#define PerformanceCriticalTask_Timer_Frequency 170000000.f
#define PerformanceCriticalTask_Period ((float)Timer_Period_ARR / PerformanceCriticalTask_Timer_Frequency * 2.f)

#define PIC_Current_Out_Limit (MC_SafeVoltage / 1.75f)
#define PIC_Current_Kp (Motor_Ld * 2000.f)
#define PIC_Current_Ki (Motor_Rs * 50.f)
#define PIC_Current_Int_Limit (PIC_Current_Out_Limit / PIC_Current_Ki)

#define PIC_Speed_Out_Limit 6.9f
#define PIC_Speed_Kp 0.01f
#define PIC_Speed_Ki 0.10f
#define PIC_Speed_Int_Limit (PIC_Speed_Out_Limit / PIC_Speed_Ki)

// #define MC_SafeVoltage 24.f

// #define Motor_PolePairs 20u
// #define Motor_Ld (0.00005f)
// #define Motor_Lq (0.00005f)
// #define Motor_Ls (0.00005f)
// #define Motor_Rs (0.05f)
// #define Motor_J (0.000001f)
// #define Motor_Flux (0.02f)

// #define ABZ_PPR 2500u

// #define Timer_Period_ARR    8499
// #define SafetyTask_Period   0.001f
// #define PerformanceCriticalTask_Timer_Frequency 170000000.f
// #define PerformanceCriticalTask_Period ((float)Timer_Period_ARR / PerformanceCriticalTask_Timer_Frequency * 2.f)

// #define PIC_Current_Out_Limit (MC_SafeVoltage / 1.85f)
// #define PIC_Current_Kp (Motor_Ld * 2000.f)
// #define PIC_Current_Ki (Motor_Rs * 50.f)
// #define PIC_Current_Int_Limit (PIC_Current_Out_Limit / PIC_Current_Ki)

// #define PIC_Speed_Out_Limit 7.0f
// #define PIC_Speed_Kp 0.01f
// #define PIC_Speed_Ki 0.10f
// #define PIC_Speed_Int_Limit (PIC_Speed_Out_Limit / PIC_Speed_Ki)

typedef struct 
{
    f32_t dat0,dat1,dat2;
}FrameForRTT;

typedef struct 
{
    f32_t dat0,dat1,dat2;
    uint8_t tail[4];
}FrameSendForUSART;

typedef struct 
{
    uint8_t receiveDat[128];
}FrameReceiveForUSART;

extern FrameReceiveForUSART frameReceiveForUSART;
extern FrameSendForUSART frameSendForUSART;

extern void Hardware_SafatyTaskEvent(void);
extern void Hardware_PerformanceTaskEvent(void);

extern void Hardware_Init(void);
extern bool Hardware_MCStartOrStop(void);
extern void Hardware_SetCCR(int32_t ccr1,int32_t ccr2,int32_t ccr3);
extern void Hardware_StartPWM(void);
extern void Hardware_StopPWM(void);
extern void Hardware_ForceSwitchOnAllLowSides(void);
extern void Hardware_ResetAllLowSides(void);

extern void Hardware_SetPulseCounter(uint32_t cnt);
extern uint32_t Hardware_GetPulseCounter(void);
extern void Hardwarre_SetABZCounter(uint32_t cnt);
extern uint32_t Hardware_GetABZCounter(void);
extern uint32_t Hardware_GetABZCounterDir(void);

extern f32_t Hardware_GetBusVoltage(void);
extern f32_t Hardware_GetTemperature(void);

extern Components2 Hardware_GetSinCosVal(f32_t angleRad);
extern f32_t Hardware_FastSquareRoot(f32_t x);
extern f32_t Hardware_FastReciprocalSquareRoot(f32_t x);

extern Components2 Harware_GetCurrentAB(void);

#endif


