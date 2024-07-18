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

extern volatile MCSysHandler mcSystemHandler;
extern volatile SvpwmHandler svpwmHandler;
extern volatile MC_MotorPar motorParHandler;
extern volatile IncABZEncoder incABZHandler;
extern volatile HFPIHandler hfpiHandler;
extern volatile HFSIHandler hfsiHandler;
extern volatile SensorHandler sensorHandler;
extern volatile PIC currentIdPICHandler;
extern volatile PIC currentIqPICHandler;
extern volatile PIC speedPICHandler;
extern volatile NSIdentifyProcessHandler NSIdendityHandler;
extern volatile MC_ParameterIdentify_Handler MCParameterIdentifyHandler;
extern volatile NSCheckHandler NSHandler;
extern volatile NonlinearFluxObsHandler NonlinearFluxHandler;

extern volatile MCSysHandler* pSys;
extern volatile SvpwmHandler* pSVP;
extern volatile MC_MotorPar* pMotor;
extern volatile IncABZEncoder* pIncABZ;
extern volatile SensorHandler* pSens;
extern volatile HFPIHandler* pHFPI;
extern volatile HFSIHandler* pHFSI;
extern volatile PIC* pIdPIC;
extern volatile PIC* pIqPIC;
extern volatile PIC* pSpPIC;
extern volatile NSIdentifyProcessHandler* pNSIdentify;
extern volatile MC_ParameterIdentify_Handler* pParmeterIndentify;
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

