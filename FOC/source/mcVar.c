/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-14 10:55:42
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-08-07 19:19:38
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\source\mcVar.c
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcVar.h"
#include "../interface/mcConfig.h"
#include "../include/mcMath.h"

volatile MCSysHandler mcSystemHandler = {
    .sysStu = eWaitSysReset
};
volatile SvpwmHandler svpwmHandler;
volatile MC_MotorPar motorParHandler;
volatile IncABZEncoder incABZHandler;
volatile SensorHandler sensorHandler;
volatile HFPIHandler hfpiHandler;
volatile HFSIHandler hfsiHandler;
volatile PIC currentIdPICHandler;
volatile PIC currentIqPICHandler;
volatile PIC speedPICHandler;
volatile NSIdentifyProcessHandler NSIdendityHandler;
volatile MC_ParameterIdentify_Handler MCParameterIdentifyHandler;
volatile NSCheckHandler NSHandler;
volatile NonlinearFluxObsHandler NonlinearFluxHandler;
volatile OpenLoop_IF_Handler IFHandler;
volatile LuenbergerObsHandler luenbergerObsHandler;
volatile AbsEncoderHandler absEncoderHandler;

volatile MCSysHandler* pSys = &mcSystemHandler;
volatile SvpwmHandler* pSVP = &svpwmHandler;
volatile MC_MotorPar* pMotor = &motorParHandler;
volatile IncABZEncoder* pIncABZ = &incABZHandler;
volatile SensorHandler* pSens = &sensorHandler;
volatile HFPIHandler* pHFPI = &hfpiHandler;
volatile HFSIHandler* pHFSI = &hfsiHandler;
volatile PIC* pIdPIC = &currentIdPICHandler;
volatile PIC* pIqPIC = &currentIqPICHandler;
volatile PIC* pSpPIC = &speedPICHandler;
volatile NSIdentifyProcessHandler* pNSIdentify = &NSIdendityHandler;
volatile MC_ParameterIdentify_Handler* pParmeterIndentify = &MCParameterIdentifyHandler;
volatile NSCheckHandler* pNS = &NSHandler;
volatile NonlinearFluxObsHandler* pNonlinearFlux = &NonlinearFluxHandler;
volatile OpenLoop_IF_Handler* pIF = &IFHandler;
volatile LuenbergerObsHandler* pLuenberger = &luenbergerObsHandler;
volatile AbsEncoderHandler* pAbs = &absEncoderHandler;

void reset_All(void)
{
    reset_MCSysHandler();
    reset_SvpwmHandler();
    reset_MotorParHandler();
    reset_SensorHandler();
    reset_HFPIHandler();
    reset_HFSIHandler();
    
    reset_CurrentPICHandler();
    reset_SpeedPICHandler();

    reset_NSIdentifyHandler();
    reset_ParmeterHandler();
    reset_NSCheckHandler();
    reset_NonlinearFluxObsHandler();
    reset_IncABZHandler();

    reset_IFHandler();
    reset_Luenberger();
    reset_AbsEncoderHandler();
}
void reset_MCSysHandler(void)
{
    mcSystemHandler.isPhaseCurrenfOffsetFinished = false;
    mcSystemHandler.sysError = eOKFlag;
    mcSystemHandler.sysStu = eWaitSysReset;
    mcSystemHandler.sysRunTime = 0u;
    mcSystemHandler.safeTaskTimeCnt = 0u;
    mcSystemHandler.focTaskTimeCnt = 0u;
    mcSystemHandler.controlMethod = eMethod_HFPI_WithoutNS;
    mcSystemHandler.focStep = eFOC_Step_1;

    mcSystemHandler.lowSpeedClock = 0.001f;
    mcSystemHandler.highSpeedClock = 0.0001f;
    mcSystemHandler.pulseSpeedClock = 1.f / 170000000.f;
}

void reset_SvpwmHandler(void)
{
    svpwmHandler.svpFrequency = PerformanceCriticalTask_Timer_Frequency;
    svpwmHandler.timerARR = Timer_Period_ARR;
    svpwmHandler.motorVoltage = MC_SafeVoltage;
    svpwmHandler.limitVoltage = MC_SafeVoltage / 1.732050807568877f;
    svpwmHandler.volAlphaBeta.com1 = 0.f;svpwmHandler.volAlphaBeta.com2 = 0.f;
    svpwmHandler.volDQ.com1 = 0.f;svpwmHandler.volDQ.com2 = 0.f;
    svpwmHandler.ccr[0] = 0;svpwmHandler.ccr[1] = 0;svpwmHandler.ccr[2] = 0;
    svpwmHandler.sector = 0;
}

void reset_MotorParHandler(void)
{
    motorParHandler.Flux = Motor_Flux;
    motorParHandler.J = Motor_J;
    motorParHandler.Ld = Motor_Ld;
    motorParHandler.Lq = Motor_Lq;
    motorParHandler.Ls = Motor_Ls;
    motorParHandler.polePairs = Motor_PolePairs;
    motorParHandler.Rs = Motor_Rs;
}

void reset_SensorHandler(void)
{
    sensorHandler.adcCorrectionCoefficient = 1.f;
    sensorHandler.sinCosVal.com1 = 0.f;sensorHandler.sinCosVal.com2 = 0.f;
    sensorHandler.busAndTemp.com1 = 0.f;sensorHandler.busAndTemp.com2 = 0.f;
    sensorHandler.currentOffset.com1 = 0.f;sensorHandler.currentOffset.com2 = 0.f;
    sensorHandler.currentAB.com1 = 0.f;sensorHandler.currentAB.com2 = 0.f;
    sensorHandler.currentDQ.com1 = sensorHandler.currentDQ.com2 = 0.f;
    sensorHandler.currentAlphaBeta.com1 = sensorHandler.currentAlphaBeta.com2 = 0.f;
}

void reset_HFPIHandler(void)
{
    hfpiHandler.injectVoltage = 0.2f;
    hfpiHandler.est_eleAngle = 0.f;hfpiHandler.est_eleSpeed = 0.f;
    hfpiHandler.inject_phase = 0.f;
    hfpiHandler.inject_phaseSinCos.com1 = 0.f;hfpiHandler.inject_phaseSinCos.com2 = 1.f;
    hfpiHandler.response_HF_iDQ.com1 = hfpiHandler.response_HF_iDQ.com2 = 0.f;
    hfpiHandler.response_iAlphaBeta.com1 = 0.f;hfpiHandler.response_iAlphaBeta.com2 = 0.f;
    hfpiHandler.response_iDQ.com1 = 0.f;hfpiHandler.response_iDQ.com2 = 0.f;
    hfpiHandler.est_err = 0.f;
    hfpiHandler.maxId = hfpiHandler.minId = 0.f;
    hfpiHandler.injectFrequency = 400.f;
    hfpiHandler.PLL_Kp = 0.1f;
    hfpiHandler.PLL_Ki = 1.f;
}

void reset_HFSIHandler(void)
{
    hfsiHandler.est_eleAngle = 0.f;hfsiHandler.est_eleSpeed = 0.f;
    hfsiHandler.inject_voltage = 0.4f;
    hfsiHandler.inject_polarity = true;
    hfsiHandler.response_HF_iAlphaBeta.com1 = 0.f;hfsiHandler.response_HF_iAlphaBeta.com2 = 0.f;
    hfsiHandler.response_HF_iDQ.com1 = 0.f;hfsiHandler.response_HF_iDQ.com2 = 0.f;
    hfsiHandler.response_iAlphaBeta.com1 = 0.f;hfsiHandler.response_iAlphaBeta.com2 = 0.f;
    hfsiHandler.response_HF_iAlphaBetaPerUnit.com1 = 0.f;hfsiHandler.response_HF_iAlphaBetaPerUnit.com2 = 0.f;
    hfsiHandler.est_err = 0.f;
    hfsiHandler.int1 = 0.f;hfsiHandler.int2 = 0.f;
    hfsiHandler.response_iDQ.com1 = hfsiHandler.response_iDQ.com2 = 0.f;
    hfsiHandler.kP = 20.f;
    hfsiHandler.kI = 0.1f;
    hfsiHandler.iAlphaBetaLast.com1 = hfsiHandler.iAlphaBetaLast.com2 = 0.f;
    hfsiHandler.iDQLast.com1 = hfsiHandler.iDQLast.com2 = 0.f;
}

void reset_CurrentPICHandler(void)
{
    currentIdPICHandler.errInt = 0.f;
    currentIqPICHandler.errInt = 0.f;
}

void reset_SpeedPICHandler(void)
{
    speedPICHandler.errInt = 0.f;
    speedPICHandler.target = 0.f;
}

void reset_NSIdentifyHandler(void)
{
    NSIdendityHandler.inject_voltage = 4.0f;
    NSIdendityHandler.inject_amp = 4.0f;
    NSIdendityHandler.posGate = 0.03f;
    NSIdendityHandler.negGate = -0.03f;
    NSIdendityHandler.polarityCntGate = 5;
    NSIdendityHandler.isCompensateFinished = false;
    NSIdendityHandler.maxId = 0.f;
    NSIdendityHandler.minId = 0.f;
    NSIdendityHandler.nsCompensate = 0.f;
    NSIdendityHandler.polarityCnt = 0;
    NSIdendityHandler.responseId = 0.f;
    NSIdendityHandler.pulseWidth = 20;
    NSIdendityHandler.pulseWidthCnt = 0;
}

void reset_ParmeterHandler(void)
{
    MCParameterIdentifyHandler.injectSigAmp = 1.f;
    MCParameterIdentifyHandler.injectFre = 200.f;
    MCParameterIdentifyHandler.mc_Ls = 0.f;
    MCParameterIdentifyHandler.mc_Rs = 0.f;
    MCParameterIdentifyHandler.demodulation_phaseCompensate = -0.058293997016611f * 7.f;
    MCParameterIdentifyHandler.demodulation_ampCompensate = 1.f;
}

void reset_NSCheckHandler(void)
{
    NSHandler.compensateAngle = 0.f;
    NSHandler.iDQ.com1 = NSHandler.iDQ.com2 = 0.f;
    NSHandler.injectPosVoltage = 3.6f;
    NSHandler.injectNegVoltage = -3.6f;
    NSHandler.injectZeroVoltage = 0.f;
    NSHandler.injectVoltage = 0.f;
    NSHandler.isFinished = false;
    NSHandler.maxId = NSHandler.minId = 0.f;
    NSHandler.pulseWidth = 20;
    NSHandler.pulseWidthCnt = 0;
    NSHandler.status = 0u;
    NSHandler.polarityCnt = 0;
} 

void reset_NonlinearFluxObsHandler(void)
{
    NonlinearFluxHandler.delay1 = NonlinearFluxHandler.delay2 = 0.f;
    NonlinearFluxHandler.integrator1 = NonlinearFluxHandler.integrator2 = 0.f;
    NonlinearFluxHandler.gamma = 160000.f;
    NonlinearFluxHandler.kP = 2000.f;
    NonlinearFluxHandler.kI = 5000.f;
    NonlinearFluxHandler.est_eleAngle = 0.f;
    NonlinearFluxHandler.est_eleSpeed = 0.f;
    NonlinearFluxHandler.integrator3 = 0.f;
    NonlinearFluxHandler.integrator4 = 0.f;
    NonlinearFluxHandler.cos = NonlinearFluxHandler.sin = 0.f;
}

void reset_IncABZHandler(void)
{
    incABZHandler.isABZEncoderAlignment = false;
    incABZHandler.isABZEncoderFinished = false;
    incABZHandler.isAlignedOK = false;
    incABZHandler.zIndexTimCnt = 0u;
    incABZHandler.lastEncoderCnt = 0u;

    incABZHandler.abzCounterMode = eABZ_X4;
    incABZHandler.encoderPPR_Uint = ABZ_PPR;
    incABZHandler.encoderPPR_XX_Uint = incABZHandler.encoderPPR_Uint * (uint32_t)incABZHandler.abzCounterMode;
    incABZHandler.eleSpeedCalculateFacotr = 2.f * MATH_PI * (f32_t)Motor_PolePairs / ((f32_t)incABZHandler.encoderPPR_XX_Uint * pSys->lowSpeedClock);
    incABZHandler.eleAngleCalculateFacotr = (f32_t)incABZHandler.encoderPPR_XX_Uint / (f32_t)Motor_PolePairs / 2.f;
    incABZHandler.realEleSpeed = 0.f;
    incABZHandler.lowEleSpeedThreshold = 2.f * MATH_PI * 20.f;
    incABZHandler.highEleSpeedThreshold = 2.f * MATH_PI * 30.f;

    if(incABZHandler.lowEleSpeedThreshold < 0.f) incABZHandler.lowEleSpeedThreshold = -incABZHandler.lowEleSpeedThreshold;
}

void reset_IFHandler(void)
{
    IFHandler.accEleSpeed = MATH_PI * 2.f * 400.f;
    IFHandler.accSpeedTime = 1000u;
    IFHandler.eleAngle = 0.f;
    IFHandler.eleSpeed = 0.f;
    IFHandler.iqRef = 4.f;
}

void reset_Luenberger(void)
{

}

void reset_AbsEncoderHandler(void)
{
    pAbs->absOffsetFromRealEleAngle = -3.2390424;
    pAbs->encoderOutput = 0u;
    pAbs->realEleAngle = 0.f;
}


