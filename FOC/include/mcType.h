/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-12 13:09:47
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-08-01 13:50:41
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\include\mcType.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _MC_TYPE_H_
#define _MC_TYPE_H_

// #ifndef CCMRAM
// #define CCMRAM __attribute__((section (".ccmram")))
// #endif

#include <stdbool.h>

typedef signed char int8_t;
typedef signed short int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;

typedef float f32_t;
typedef double f64_t;

typedef enum{
    eMethod_IncABZ = 0,
    eMethod_AbsABZ = 1,
    eMethod_ParIdentify = 2,
    eMethod_NonlinearFlux = 3,
    eMethod_NonlinearFlux_Debug = 4,
    eMethod_IF_Luenberger = 5,
    eMethod_IF_Luenberger_Debug = 6,
    eMethod_HFPI_WithoutNS = 7
}MC_ControlMethod;

typedef enum{
    eWaitSysReset = -1,
    eWaitBusVoltage = 0,
    eWaitCapCharge = 1,
    eWaitCalADCOffset = 2,
    eWaitMCStart = 3,
    eSysRun = 4
}MC_SysStateMachine;

typedef enum{
    eOKFlag = 0,
    eOverVoltageError = 1,
    eUnderVoltageError = 2
}MC_SysErrorFlag;

typedef struct{
    f32_t com1,com2,com3;
}Components3;

typedef struct{
    f32_t com1,com2;
}Components2;

typedef struct{
    f32_t adcCorrectionCoefficient;
    Components2 sinCosVal;
    Components2 busAndTemp;
    Components2 currentOffset;
    Components2 currentAB;
    Components2 currentDQ;
    Components2 currentAlphaBeta;
}SensorHandler;

typedef struct{
    uint8_t polePairs;
    f32_t Ld,Lq,Rs,Ls;
    f32_t J,Flux;
}MC_MotorPar;

typedef enum{
    eABZ_X4 = 4,
    eABZ_X2 = 2,
    eABZ_X1 = 1
}ABZCounterMode;

typedef struct
{
    bool isABZEncoderAlignment;
    bool isABZEncoderFinished;
    bool isAlignedOK;

    ABZCounterMode abzCounterMode;

    int32_t encoderPPR_XX_Uint;
    int32_t encoderPPR_Uint;

    f32_t eleSpeedCalculateFacotr;
    f32_t eleAngleCalculateFacotr;
    uint32_t zIndexTimCnt;
    uint32_t lastEncoderCnt;

    f32_t realEleSpeed,realEleAngle,lowEleSpeedThreshold,highEleSpeedThreshold;
}IncABZEncoder;

typedef struct 
{
    f32_t motorVoltage;
    f32_t limitVoltage;
    Components2 volDQ;
    Components2 volAlphaBeta;

    f32_t svpFrequency;
    uint32_t timerARR;
    uint16_t ccr[3];
    int8_t sector;
}SvpwmHandler;

typedef enum{
    eFOC_Step_1 = 0,
    eFOC_Step_2 = 1,
    eFOC_Step_3 = 2,
    eFOC_Step_4 = 3,
    eFOC_Step_5 = 4,
    eFOC_Step_6 = 5
}MC_FocStep;

typedef struct 
{
    uint32_t safeTaskTimeCnt;
    uint32_t focTaskTimeCnt;
    MC_SysStateMachine sysStu;
    MC_SysErrorFlag sysError;
    uint32_t sysRunTime;
    MC_ControlMethod controlMethod;
    MC_FocStep focStep;
    f32_t lowSpeedClock,highSpeedClock,pulseSpeedClock;

    bool isPhaseCurrenfOffsetFinished;
}MCSysHandler;

typedef struct 
{
    f32_t injectVoltage;
    Components2 inject_phaseSinCos;
    Components2 response_iAlphaBeta;
    Components2 response_iDQ;
    Components2 response_HF_iDQ;
    f32_t maxId,minId;
    f32_t inject_phase;
    f32_t est_eleAngle;
    f32_t est_eleSpeed;
    f32_t est_err;
    f32_t injectFrequency;
    f32_t PLL_Kp,PLL_Ki;
}HFPIHandler;

typedef struct 
{
    Components2 iAlphaBetaLast,iDQLast;
    Components2 response_iAlphaBeta;
    Components2 response_iDQ;
    Components2 response_HF_iDQ;
    Components2 response_HF_iAlphaBeta;
    Components2 response_LF_iDQ;
    Components2 response_LF_iAlphaBeta;
    Components2 response_HF_iAlphaBetaPerUnit;
    f32_t inject_voltage;   
    f32_t int1,int2;
    f32_t kP,kI;
    f32_t est_eleAngle;
    f32_t est_eleSpeed;
    f32_t est_err;
    bool inject_polarity;
}HFSIHandler;

typedef struct{
    f32_t target;
    f32_t kP,kI;
    f32_t errInt;
    f32_t output;
}PIC;

typedef struct {
    f32_t inject_voltage,inject_amp,responseId;
    f32_t posGate,negGate;
    f32_t maxId,minId;
    int32_t pulseWidthCnt;
    int32_t pulseWidth;
    int32_t polarityCnt,polarityCntGate;
    f32_t nsCompensate;
    bool isCompensateFinished;
}NSIdentifyProcessHandler;

typedef struct
{
    uint8_t status;
    f32_t maxId,minId;
    f32_t injectPosVoltage,injectNegVoltage,injectZeroVoltage,injectVoltage;
    f32_t compensateAngle;
    int32_t pulseWidth,pulseWidthCnt,polarityCnt;
    Components2 iDQ;
    bool isFinished;
}NSCheckHandler;

typedef struct{
    f32_t injectFre;
    f32_t injectSigAmp;
    f32_t demodulation_Phase;
    f32_t demodulation_ampCompensate,demodulation_phaseCompensate;

    Components2 demodulation_sinCos;
    Components2 sig_HF,sig_LF,sig;
    f32_t z,phase;

    f32_t mc_Rs,mc_Ls;
}MC_ParameterIdentify_Handler;

typedef struct 
{
    f32_t gamma;
    f32_t delay1,delay2;
    f32_t integrator1,integrator2;

    f32_t sin,cos;

    f32_t err;
    f32_t kP,kI;
    f32_t est_eleSpeed,est_eleAngle;
    f32_t integrator3,integrator4;
}NonlinearFluxObsHandler;

typedef struct
{
    f32_t accEleSpeed,eleSpeed,eleAngle,iqRef;
    uint32_t accSpeedTime;
}OpenLoop_IF_Handler;

typedef struct 
{
    f32_t xxxx;
}LuenbergerObsHandler;

#endif

