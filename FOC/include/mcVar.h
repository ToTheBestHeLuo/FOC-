/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-07-04 09:16:17
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-07-15 10:04:24
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\include\mcVar.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */

#ifndef _MC_VAR_H_
#define _MC_VAR_H_

#include "mcType.h"

CCMRAM extern volatile MCSysHandler mcSystemHandler;
CCMRAM extern volatile SvpwmHandler svpwmHandler;
CCMRAM extern volatile MC_MotorPar motorParHandler;
CCMRAM extern volatile IncABZEncoder incABZHandler;
extern volatile HFPIHandler hfpiHandler;
extern volatile HFSIHandler hfsiHandler;
CCMRAM extern volatile SensorHandler sensorHandler;
CCMRAM extern volatile PIC currentIdPICHandler;
CCMRAM extern volatile PIC currentIqPICHandler;
CCMRAM extern volatile PIC speedPICHandler;
extern volatile NSIdentifyProcessHandler NSIdendityHandler;
CCMRAM extern volatile MC_ParameterIdentify_Handler MCParameterIdentifyHandler;
extern volatile NSCheckHandler NSHandler;
extern volatile NonlinearFluxObsHandler NonlinearFluxHandler;

CCMRAM extern volatile MCSysHandler* pSys;
CCMRAM extern volatile SvpwmHandler* pSVP;
CCMRAM extern volatile MC_MotorPar* pMotor;
CCMRAM extern volatile IncABZEncoder* pIncABZ;
CCMRAM extern volatile SensorHandler* pSens;
extern volatile HFPIHandler* pHFPI;
extern volatile HFSIHandler* pHFSI;
CCMRAM extern volatile PIC* pIdPIC;
CCMRAM extern volatile PIC* pIqPIC;
CCMRAM extern volatile PIC* pSpPIC;
extern volatile NSIdentifyProcessHandler* pNSIdentify;
CCMRAM extern volatile MC_ParameterIdentify_Handler* pParmeterIndentify;
extern volatile NSCheckHandler* pNS;
extern volatile NonlinearFluxObsHandler* pNonlinearFlux;

extern void reset_All(void);
extern void reset_MCSysHandler(void);
extern void reset_SvpwmHandler(void);
extern void reset_MotorParHandler(void);
extern void reset_IncABZHandler(void);
extern void reset_SensorHandler(void);
extern void reset_HFPIHandler(void);
extern void reset_HFSIHandler(void);
extern void reset_CurrentPICHandler(void);
extern void reset_SpeedPICHandler(void);
extern void reset_NSIdentifyHandler(void);
extern void reset_ParmeterHandler(void);
extern void reset_NSCheckHandler(void);
extern void reset_NonlinearFluxObsHandler(void);
#endif

