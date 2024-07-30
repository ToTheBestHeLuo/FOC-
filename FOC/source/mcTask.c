/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-14 10:55:42
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-07-30 14:41:09
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\source\mcTask.c
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcTask.h"
#include "../interface/mcConfig.h"
#include "../include/mcMath.h"
#include "../include/mcVar.h"
#include "../include/mcParIdentify.h"
#include "../include/mcHVSensorless.h"
#include "../include/mcLVSensorless.h"
#include "../include/mcSensor.h"

#define sqrt3 1.732050807568877f

static f32_t adcOffsetFilterA[100] = {0.f};
static f32_t adcOffsetFilterB[100] = {0.f};

const uint32_t lengthCurrentFilter = sizeof(adcOffsetFilterA) / sizeof(f32_t);

void SectorCalModeSvpwm(volatile SvpwmHandler* svp,f32_t bus);
Components2 CurrentPIController(volatile PIC* idPIC,volatile PIC* iqPIC,volatile Components2* idqReal);
f32_t SpeedPIController(volatile PIC* speedPIC,f32_t realSpeed);

void FOC_Method_IncABZ(void);
void FOC_Method_ParIdentify(void);
void FOC_Method_NonlinearFlux(void);
void FOC_Method_NonlinearFlux_Debug(void);
void FOC_Method_IF_Luenberger(void);
void FOC_Method_IF_Luenberger_Debug(void);
void FOC_Method_HFPI_WithoutNS(void);

void SectorVoltageLimit(volatile SvpwmHandler* svp,Components2* svpAlphaBeta)
{
    f32_t tmp = svpAlphaBeta->com1 * svpAlphaBeta->com1 + svpAlphaBeta->com2 * svpAlphaBeta->com2;
    if(tmp > svp->limitVoltage * svp->limitVoltage){
        
    }
}

void SectorCalModeSvpwm(volatile SvpwmHandler* svp,f32_t bus)
{
    f32_t period = pSys->highSpeedClock;

    f32_t u1,u2,u3,tx,ty;
    int8_t cnt = 0;
    f32_t volAlpha,volBeta;
    volAlpha = svp->volAlphaBeta.com1;
    volBeta = svp->volAlphaBeta.com2;
    u1 = volBeta;
    u2 = sqrt3 / 2.f * volAlpha - 0.5f * volBeta;
    u3 = -sqrt3 / 2.f * volAlpha - 0.5f * volBeta;
    f32_t cons = sqrt3 * period / bus;

    if(u1 > 0.f) cnt += 1;
    if(u2 > 0.f) cnt += 2;
    if(u3 > 0.f) cnt += 4;

    f32_t ta,tb,tc;

    switch(cnt)
    {
        case 1:
            svp->sector = 2;
            tx = cons * (-sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ty = cons * (sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ta = (period - tx + ty) * 0.25f;
            tb = (period + tx + ty) * 0.25f;
            tc = (period - tx - ty) * 0.25f;
            break;
        case 2:
            svp->sector = 6;
            tx = cons * (sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ty = -cons * (volBeta);
            ta = (period + tx + ty) * 0.25f;
            tb = (period - tx - ty) * 0.25f;
            tc = (period - tx + ty) * 0.25f;
            break;
        case 3:
            svp->sector = 1;
            tx = cons * (sqrt3 / 2.f * volAlpha - 0.5f * volBeta);
            ty = cons * (volBeta);
            ta = (period + tx + ty) * 0.25f;
            tb = (period - tx + ty) * 0.25f;
            tc = (period - tx - ty) * 0.25f;
            break;
        case 4:
            svp->sector = 4;
            tx = -cons * (volBeta);
            ty = cons * (-sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ta = (period - tx - ty) * 0.25f;
            tb = (period - tx + ty) * 0.25f;
            tc = (period + tx + ty) * 0.25f;
            break;
        case 5:
            svp->sector = 3;
            tx = cons * (volBeta);
            ty = -cons * (sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ta = (period - tx - ty) * 0.25f;
            tb = (period + tx + ty) * 0.25f;
            tc = (period - tx + ty) * 0.25f;
            break;
        case 6:
            svp->sector = 5;
            tx = -cons * (sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ty = -cons * (-sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ta = (period - tx + ty) * 0.25f;
            tb = (period - tx - ty) * 0.25f;
            tc = (period + tx + ty) * 0.25f;
            break;
        default:
            ta = (period) * 0.25f;
            tb = (period) * 0.25f;
            tc = (period) * 0.25f;
            break;
    }

    int32_t ccr1,ccr2,ccr3;
    
    ccr1 = ta * svp->svpFrequency;
    ccr2 = tb * svp->svpFrequency;
    ccr3 = tc * svp->svpFrequency;

    if(ccr1 < 0) ccr1 = 0;
    else if(ccr1 > svp->timerARR) ccr1 = svp->timerARR;
    if(ccr2 < 0) ccr2 = 0;
    else if(ccr2 > svp->timerARR) ccr2 = svp->timerARR;
    if(ccr3 < 0) ccr3 = 0;
    else if(ccr3 > svp->timerARR) ccr3 = svp->timerARR;

    svp->ccr[0] = ccr1;
    svp->ccr[1] = ccr2;
    svp->ccr[2] = ccr3;
}

void SafetyTask(void)
{
    static uint64_t sysRunTimeCnt = 0;
    static f32_t busFilter[10] = {0.f};
    const uint32_t lengthBusFilter = sizeof(busFilter) / sizeof(f32_t);
    switch(pSys->sysStu){
        case eWaitSysReset:
            Hardware_StopPWM();
            reset_All();
            pSys->sysStu = eWaitBusVoltage;
            break;
        case eWaitBusVoltage:
            busFilter[sysRunTimeCnt % lengthBusFilter] = Hardware_GetBusVoltage();
            pSens->busAndTemp.com1 = MedianFilter(busFilter,lengthBusFilter);
            if(pSens->busAndTemp.com1 > MC_SafeVoltage * 0.85f && pSens->busAndTemp.com1 < MC_SafeVoltage * 1.15f){
                if(pSys->safeTaskTimeCnt++ > 50u){
                    Hardware_ForceSwitchOnAllLowSides();
                    pSys->safeTaskTimeCnt = 0;
                    pSys->sysStu = eWaitCapCharge;            
                }
            }
            else{pSys->safeTaskTimeCnt = 0;}
            break;
        case eWaitCapCharge:
            if(pSys->safeTaskTimeCnt++ > 50u){
                pSys->focTaskTimeCnt = 0;
                pSys->safeTaskTimeCnt = 0;
                pSys->sysStu = eWaitCalADCOffset;
            }
            else if(pSys->safeTaskTimeCnt > 45u){
                uint32_t ccr = pSVP->timerARR;
                Hardware_SetCCR(ccr,ccr,ccr);
                Hardware_StartPWM();
            }
            else if(pSys->safeTaskTimeCnt > 35u){
                Hardware_ResetAllLowSides();
            }
            break;
        case eWaitCalADCOffset:
            if(pSys->isPhaseCurrenfOffsetFinished){
                pSens->currentOffset.com1 = MedianFilter(adcOffsetFilterA,lengthCurrentFilter);
                pSens->currentOffset.com2 = MedianFilter(adcOffsetFilterB,lengthCurrentFilter);
                pSys->safeTaskTimeCnt = 0;
                pSys->sysStu = eWaitMCStart;
            }
            break;
        case eWaitMCStart:
            if(Hardware_MCStartOrStop()){
                Hardware_StartPWM();
                pSys->sysStu = eSysRun;
            }
            break;
        default:
            break;
    }
    if(pSys->sysStu == eSysRun){
        if(!Hardware_MCStartOrStop()){pSys->sysStu = eWaitSysReset;}
        busFilter[sysRunTimeCnt % (sizeof(busFilter) / sizeof(f32_t))] = Hardware_GetBusVoltage();
        pSens->busAndTemp.com1 = MedianFilter(busFilter,(sizeof(busFilter) / sizeof(f32_t)));
        if(pSens->busAndTemp.com1 < MC_SafeVoltage * 0.85f){
            if(pSys->safeTaskTimeCnt++ > 140u){
                Hardware_StopPWM();
                pSys->safeTaskTimeCnt = 0;
                pSys->sysError = eUnderVoltageError;
                pSys->sysStu = eWaitSysReset;
            }
        }
        else if(pSens->busAndTemp.com1 > MC_SafeVoltage * 1.6f){
            if(pSys->safeTaskTimeCnt++ > 140u){
                Hardware_StopPWM();
                pSys->safeTaskTimeCnt = 0;
                pSys->sysError = eOverVoltageError;
                pSys->sysStu = eWaitSysReset;
            }
        }
        else{pSys->safeTaskTimeCnt = 0;}
    }

    sysRunTimeCnt++;

    Hardware_SafatyTaskEvent();
}

f32_t SpeedPIController(volatile PIC* speedPIC,f32_t realSpeed)
{
    f32_t ts = pSys->lowSpeedClock;

    f32_t out;
    f32_t err = speedPIC->target - realSpeed;

    speedPIC->errInt += err * ts;

    if(speedPIC->errInt > PIC_Speed_Int_Limit) speedPIC->errInt = PIC_Speed_Int_Limit;
    else if(speedPIC->errInt < -PIC_Speed_Int_Limit) speedPIC->errInt = -PIC_Speed_Int_Limit;

    out = speedPIC->errInt * PIC_Speed_Ki + err * PIC_Speed_Kp;

    if(out > PIC_Speed_Out_Limit) out = PIC_Speed_Out_Limit;
    else if(out < -PIC_Speed_Out_Limit) out = -PIC_Speed_Out_Limit;

    speedPIC->output = out;

    return out;
}

Components2 CurrentPIController(volatile PIC* idPIC,volatile PIC* iqPIC,volatile Components2* idqReal)
{
    f32_t ts = pSys->highSpeedClock;

    Components2 out;
    f32_t err = idPIC->target - idqReal->com1;
    idPIC->errInt += err * ts;

    if(idPIC->errInt > PIC_Current_Int_Limit) idPIC->errInt = PIC_Current_Int_Limit;
    else if(idPIC->errInt < -PIC_Current_Int_Limit) idPIC->errInt = -PIC_Current_Int_Limit;

    out.com1 = idPIC->errInt * PIC_Current_Ki + err * PIC_Current_Kp;

    if(out.com1 > PIC_Current_Out_Limit) out.com1 = PIC_Current_Out_Limit;
    else if(out.com1 < -PIC_Current_Out_Limit) out.com1 = -PIC_Current_Out_Limit;

    err = iqPIC->target - idqReal->com2;
    iqPIC->errInt += err * ts;

    if(iqPIC->errInt > PIC_Current_Int_Limit) iqPIC->errInt = PIC_Current_Int_Limit;
    else if(iqPIC->errInt < -PIC_Current_Int_Limit) iqPIC->errInt = -PIC_Current_Int_Limit;

    out.com2 = iqPIC->errInt * PIC_Current_Ki + err * PIC_Current_Kp;

    if(out.com2 > PIC_Current_Out_Limit) out.com2 = PIC_Current_Out_Limit;
    else if(out.com2 < -PIC_Current_Out_Limit) out.com2 = -PIC_Current_Out_Limit;

    idPIC->output = out.com1;
    iqPIC->output = out.com2;

    return out;
}

void PerformanceCriticalTask(void)
{
    pSens->currentAB = Harware_GetCurrentAB();
    if(pSys->sysStu == eSysRun){ 
        pSens->currentAB.com1 -= pSens->currentOffset.com1;
        pSens->currentAB.com2 -= pSens->currentOffset.com2;
        switch(pSys->controlMethod){
            case eMethod_AbsABZ:
                break;
            case eMethod_IncABZ:
                FOC_Method_IncABZ();
                break;
            case eMethod_ParIdentify:
                FOC_Method_ParIdentify();
                break;
            case eMethod_NonlinearFlux:
                FOC_Method_NonlinearFlux();
                break;
            case eMethod_NonlinearFlux_Debug:
                FOC_Method_NonlinearFlux_Debug();
                break;
            case eMethod_IF_Luenberger:
                FOC_Method_IF_Luenberger();
                break;
            case eMethod_IF_Luenberger_Debug:
                FOC_Method_IF_Luenberger_Debug();
                break;
            case eMethod_HFPI_WithoutNS:
                FOC_Method_HFPI_WithoutNS();
                break;
            default:
                break;
        }
        pSVP->volAlphaBeta = Dq_AlphaBeta_Trans(&pSVP->volDQ,&pSens->sinCosVal);
        SectorCalModeSvpwm(pSVP,pSens->busAndTemp.com1);
        Hardware_SetCCR(pSVP->ccr[0],pSVP->ccr[1],pSVP->ccr[2]);
    }else if(pSys->sysStu == eWaitCalADCOffset && !pSys->isPhaseCurrenfOffsetFinished){
        if(pSys->focTaskTimeCnt++ != lengthCurrentFilter){
            adcOffsetFilterA[pSys->focTaskTimeCnt] = pSens->currentAB.com1;
            adcOffsetFilterB[pSys->focTaskTimeCnt] = pSens->currentAB.com2;
        }else{
            pSys->focTaskTimeCnt = 0;
            pSys->isPhaseCurrenfOffsetFinished = true;
        }
    }
    Hardware_PerformanceTaskEvent();
}

void FOC_Method_IncABZ(void)
{
    Components2 iAlphaBeta,uAlphaBeta;
    Components2 piOut;
    f32_t realSpeed = pIncABZ->realEleSpeed;
    switch(pSys->focStep){
        case eFOC_Step_1:
            if(pSys->focTaskTimeCnt++ < 2000){
                pIdPIC->target = 4.f;
                pIqPIC->target = 0.f;
                pSens->sinCosVal = Hardware_GetSinCosVal(0.f);
                iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
                pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
                piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
                pSVP->volDQ.com1 = piOut.com1;
                pSVP->volDQ.com2 = piOut.com2;
            }
            else{
                pSys->focTaskTimeCnt = 0;
                pIdPIC->target = 0.f;
                pSVP->volDQ.com1 = pSVP->volDQ.com2 = 0.f;
                pIncABZ->isABZEncoderAlignment = true;
                pIncABZ->lastEncoderCnt = 1250;
                reset_CurrentPICHandler();
                Hardwarre_SetABZCounter(1250);
                pSys->focStep = eFOC_Step_2;
            }
            break;
        case eFOC_Step_2:
            if(pSys->focTaskTimeCnt++ > 100){Hardware_SetPulseCounter(0);pSys->focTaskTimeCnt = 0;pSys->focStep = eFOC_Step_3;}
            break;
        case eFOC_Step_3:
            pIncABZ->realEleAngle = IncAbzCalculateRealEleAngle(pIncABZ);
            pSens->sinCosVal = Hardware_GetSinCosVal(pIncABZ->realEleAngle);
            iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
            pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
            piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
            piOut.com1 = piOut.com1 - pSens->currentDQ.com2 * pMotor->Lq * realSpeed;
            piOut.com2 = piOut.com2 + (pSens->currentDQ.com1 * pMotor->Ld + pMotor->Flux) * realSpeed;
            pSVP->volDQ.com1 = piOut.com1;
            pSVP->volDQ.com2 = piOut.com2;
            if(pSys->focTaskTimeCnt++ == 9u){
                pSys->focTaskTimeCnt = 0u;
                pIncABZ->realEleSpeed = IncAbzCalculateRealEleSpeed(pIncABZ,pIncABZ->realEleSpeed);
                pIqPIC->target = SpeedPIController(pSpPIC,pIncABZ->realEleSpeed);
            }
            break;
        default:
            break;
    }
}

void FOC_Method_IF_Luenberger(void)
{
    Components2 iAlphaBeta,piOut;
    switch(pSys->focStep){
        case eFOC_Step_1:
            if(pSys->focTaskTimeCnt++ < 2000){
                pIdPIC->target = pIF->iqRef;
                pIqPIC->target = 0.f;
                pSens->sinCosVal = Hardware_GetSinCosVal(0.f);
                iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
                pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
                piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
                pSVP->volDQ.com1 = piOut.com1;
                pSVP->volDQ.com2 = piOut.com2;
            }
            else{
                pSys->focTaskTimeCnt = 0;
                pIdPIC->target = 0.f;
                pSVP->volDQ.com1 = pSVP->volDQ.com2 = 0.f;
                reset_CurrentPICHandler();
                pSys->focStep = eFOC_Step_2;
            }
            break;
        case eFOC_Step_2:
            if(pSys->focTaskTimeCnt++ > 100){pIqPIC->target = pIF->iqRef;
            pSys->focTaskTimeCnt = 0;pSys->focStep = eFOC_Step_3;}
            break;
        case eFOC_Step_3:
            pSens->sinCosVal = Hardware_GetSinCosVal(pIF->eleAngle);
            iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
            pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
            piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
            pSVP->volDQ.com1 = piOut.com1;
            pSVP->volDQ.com2 = piOut.com2;
            if(pSys->focTaskTimeCnt++ < pIF->accSpeedTime){pIF->eleSpeed += pSys->highSpeedClock * pIF->accEleSpeed;}
            else{pSys->focTaskTimeCnt = 0;pSys->focStep = eFOC_Step_4;}
            pIF->eleAngle += pSys->highSpeedClock * pIF->eleSpeed;
            if(pIF->eleAngle > MATH_PI){pIF->eleAngle = pIF->eleAngle - 2.f * MATH_PI;}
            else if(pIF->eleAngle < -MATH_PI){pIF->eleAngle = pIF->eleAngle + 2.f * MATH_PI;}
        case eFOC_Step_4:
            pSens->sinCosVal = Hardware_GetSinCosVal(pIF->eleAngle);
            iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
            pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
            piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
            pSVP->volDQ.com1 = piOut.com1;
            pSVP->volDQ.com2 = piOut.com2;
            pIF->eleAngle += pSys->highSpeedClock * pIF->eleSpeed;
            if(pIF->eleAngle > MATH_PI){pIF->eleAngle = pIF->eleAngle - 2.f * MATH_PI;}
            else if(pIF->eleAngle < -MATH_PI){pIF->eleAngle = pIF->eleAngle + 2.f * MATH_PI;}
            break;
        default:
            break;
    }
}

void FOC_Method_IF_Luenberger_Debug(void)
{
    Components2 iAlphaBeta,piOut;
    switch(pSys->focStep){
        case eFOC_Step_1:
            if(pSys->focTaskTimeCnt++ < 2000){
                pIdPIC->target = pIF->iqRef;
                pIqPIC->target = 0.f;
                pSens->sinCosVal = Hardware_GetSinCosVal(0.f);
                iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
                pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
                piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
                pSVP->volDQ.com1 = piOut.com1;
                pSVP->volDQ.com2 = piOut.com2;
                Hardwarre_SetABZCounter(1250);
            }
            else{
                pSys->focTaskTimeCnt = 0;
                pIdPIC->target = 0.f;
                pSVP->volDQ.com1 = pSVP->volDQ.com2 = 0.f;
                reset_CurrentPICHandler();
                pIncABZ->isABZEncoderAlignment = true;
                pIncABZ->lastEncoderCnt = 1250;
                pSys->focStep = eFOC_Step_2;
            }
            break;
        case eFOC_Step_2:
            if(pSys->focTaskTimeCnt++ > 100){pIqPIC->target = pIF->iqRef;
            pSys->focTaskTimeCnt = 0;pSys->focStep = eFOC_Step_3;}
            break;
        case eFOC_Step_3:
            pSens->sinCosVal = Hardware_GetSinCosVal(pIF->eleAngle);
            iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
            pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
            piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
            pSVP->volDQ.com1 = piOut.com1;
            pSVP->volDQ.com2 = piOut.com2;
            if(pSys->focTaskTimeCnt++ < pIF->accSpeedTime){pIF->eleSpeed += pSys->highSpeedClock * pIF->accEleSpeed;}
            else{pSys->focTaskTimeCnt = 0;pSys->focStep = eFOC_Step_4;}
            pIF->eleAngle += pSys->highSpeedClock * pIF->eleSpeed;
            if(pIF->eleAngle > MATH_PI){pIF->eleAngle = pIF->eleAngle - 2.f * MATH_PI;}
            else if(pIF->eleAngle < -MATH_PI){pIF->eleAngle = pIF->eleAngle + 2.f * MATH_PI;}
        case eFOC_Step_4:
            pIncABZ->realEleAngle = IncAbzCalculateRealEleAngle(pIncABZ);
            pSens->sinCosVal = Hardware_GetSinCosVal(pIF->eleAngle);
            iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
            pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
            piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
            pSVP->volDQ.com1 = piOut.com1;
            pSVP->volDQ.com2 = piOut.com2;
            pIF->eleAngle += pSys->highSpeedClock * pIF->eleSpeed;
            if(pIF->eleAngle > MATH_PI){pIF->eleAngle = pIF->eleAngle - 2.f * MATH_PI;}
            else if(pIF->eleAngle < -MATH_PI){pIF->eleAngle = pIF->eleAngle + 2.f * MATH_PI;}
            break;
        default:
            break;
    }
}

void FOC_Method_ParIdentify(void)
{
    static f32_t polarity = 1.f;
    Components2 iAlphaBeta,piOut;
    switch(pSys->focStep){
        case eFOC_Step_1:
            if(pSys->focTaskTimeCnt++ < 2000){
                pIdPIC->target = 4.f;
                pIqPIC->target = 0.f;
                pSens->sinCosVal = Hardware_GetSinCosVal(MATH_PI * 0.f / 180.f);
                iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
                pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
                piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
                pSVP->volDQ.com1 = piOut.com1;
                pSVP->volDQ.com2 = piOut.com2;
            }
            else{
                pSVP->volDQ.com1 = 0.f;
                pSVP->volDQ.com2 = 0.f;
                reset_CurrentPICHandler();
                pSys->focTaskTimeCnt = 0;
                pSys->focStep = eFOC_Step_2;
            }
            break;
        case eFOC_Step_2:
            if(pSys->focTaskTimeCnt++ > 100){pSys->focTaskTimeCnt = 0;pSys->focStep = eFOC_Step_3;}
            break;
        case eFOC_Step_3:
            iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
            pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
            if(pSys->focTaskTimeCnt == 25u){pSys->focTaskTimeCnt = 0u;polarity = -polarity;}
            if(pSys->focTaskTimeCnt++ == 0u){pSVP->volDQ.com1 = -pParmeterIndentify->injectSigAmp * polarity + 0.2f;}
            MCParIdentify_Rs_Ls(pParmeterIndentify,pSens->currentDQ.com1);
            break;
        default:
            break;
    }
}

void FOC_Method_NonlinearFlux(void)
{
    f32_t realEleSpeed;
    Components2 iAlphaBeta,piOut,uAlphaBeta;
    switch(pSys->focStep){
        case eFOC_Step_1:
            pIdPIC->target = 0.f;
            pSys->focStep = eFOC_Step_2;
            break;
        case eFOC_Step_2:
            pSens->sinCosVal = Hardware_GetSinCosVal(pNonlinearFlux->est_eleAngle);
            iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
            pSens->currentAlphaBeta = iAlphaBeta;
            pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
            piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
            piOut.com1 = piOut.com1 - pSens->currentDQ.com2 * pMotor->Lq * pNonlinearFlux->est_eleSpeed;
            piOut.com2 = piOut.com2 + (pSens->currentDQ.com1 * pMotor->Ld + pMotor->Flux) * pNonlinearFlux->est_eleSpeed;
            pSVP->volDQ.com1 = piOut.com1;
            pSVP->volDQ.com2 = piOut.com2;
            uAlphaBeta = Dq_AlphaBeta_Trans(&piOut,&pSens->sinCosVal);
            NonlinearFluxObsProcess(pMotor,pNonlinearFlux,pSpPIC,&uAlphaBeta,&iAlphaBeta);
            if(pSys->focTaskTimeCnt++ == 9u){
                pSys->focTaskTimeCnt = 0u;
                pIqPIC->target = SpeedPIController(pSpPIC,pNonlinearFlux->est_eleSpeed);
            }
            break;
        default:
            break;
    }
}

void FOC_Method_NonlinearFlux_Debug(void)
{
    Components2 iAlphaBeta,uAlphaBeta;
    Components2 piOut;
    f32_t realEleSpeed;
    switch(pSys->focStep){
        case eFOC_Step_1:
            if(pSys->focTaskTimeCnt++ < 20000){
                pIdPIC->target = pIF->iqRef;
                pIqPIC->target = 0.f;
                pSens->sinCosVal = Hardware_GetSinCosVal(0.f);
                iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
                pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
                piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
                pSVP->volDQ.com1 = piOut.com1;
                pSVP->volDQ.com2 = piOut.com2;
            }
            else{
                pSys->focTaskTimeCnt = 0;
                pIdPIC->target = 0.f;
                pSVP->volDQ.com1 = pSVP->volDQ.com2 = 0.f;
                pIncABZ->isABZEncoderAlignment = true;
                pIncABZ->lastEncoderCnt = 1250;
                reset_CurrentPICHandler();
                Hardwarre_SetABZCounter(1250);
                pSys->focStep = eFOC_Step_2;
            }
            break;
        case eFOC_Step_2:
            if(pSys->focTaskTimeCnt++ > 100){Hardware_SetPulseCounter(0);pSys->focTaskTimeCnt = 0;pSys->focStep = eFOC_Step_3;}
            break;
        case eFOC_Step_3:
            pIncABZ->realEleAngle = IncAbzCalculateRealEleAngle(pIncABZ);
            pSens->sinCosVal = Hardware_GetSinCosVal(pNonlinearFlux->est_eleAngle);
            iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
            pSens->currentAlphaBeta = iAlphaBeta;
            pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
            piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
            piOut.com1 = piOut.com1 - pSens->currentDQ.com2 * pMotor->Lq * pNonlinearFlux->est_eleSpeed;
            piOut.com2 = piOut.com2 + (pSens->currentDQ.com1 * pMotor->Ld + pMotor->Flux) * pNonlinearFlux->est_eleSpeed;
            pSVP->volDQ.com1 = piOut.com1;
            pSVP->volDQ.com2 = piOut.com2;
            uAlphaBeta = Dq_AlphaBeta_Trans(&piOut,&pSens->sinCosVal);
            NonlinearFluxObsProcess(pMotor,pNonlinearFlux,pSpPIC,&uAlphaBeta,&iAlphaBeta);
            if(pSys->focTaskTimeCnt++ == 9u){
                pSys->focTaskTimeCnt = 0u;
                realEleSpeed = pNonlinearFlux->est_eleSpeed;
                pIqPIC->target = SpeedPIController(pSpPIC,realEleSpeed);
            }
            break;
        default:
            break;
    } 
}

void FOC_Method_HFPI_WithoutNS(void)
{
    Components2 iAlphaBeta,piOut;
    f32_t injectVolD;
    switch(pSys->focStep){
        case eFOC_Step_1:
            if(pSys->focTaskTimeCnt++ < 2000){
                pIdPIC->target = 4.f;
                pIqPIC->target = 0.f;
                pSens->sinCosVal = Hardware_GetSinCosVal(0.f);
                iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
                pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
                piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
                pSVP->volDQ.com1 = piOut.com1;
                pSVP->volDQ.com2 = piOut.com2;
            }
            else{
                pSys->focTaskTimeCnt = 0;
                pIdPIC->target = 0.f;
                pSVP->volDQ.com1 = pSVP->volDQ.com2 = 0.f;
                reset_CurrentPICHandler();
                pSys->focStep = eFOC_Step_2;
            }
            break;
        case eFOC_Step_2:
            pSens->sinCosVal = Hardware_GetSinCosVal(0.f);
            injectVolD = HFPISensorlessObserver(pSens,pHFPI);
            pSVP->volDQ.com1 = injectVolD;
            pSVP->volDQ.com2 = 0.f;
            break;
        default:
            break;
    }
}


