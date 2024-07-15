/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-14 10:55:42
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-07-15 14:29:26
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
#include "../include/mcHFI.h"
#include "../include/mcFluxObs.h"

#define sqrt3 1.732050807568877f

CCMRAM void SectorCalModeSvpwm(volatile SvpwmHandler* svp);
CCMRAM Components2 CurrentPIController(volatile PIC* idPIC,volatile PIC* iqPIC,volatile Components2* idqReal);
CCMRAM f32_t SpeedPIController(volatile PIC* speedPIC,f32_t realSpeed);
CCMRAM void FOC_Method_IncABZ(void);
CCMRAM void FOC_Method_ParIdentify(void);

void SectorVoltageLimit(volatile SvpwmHandler* svp,Components2* svpAlphaBeta)
{
    f32_t tmp = svpAlphaBeta->com1 * svpAlphaBeta->com1 + svpAlphaBeta->com2 * svpAlphaBeta->com2;
    if(tmp > svp->limitVoltage * svp->limitVoltage){
        
    }
}

void SectorCalModeSvpwm(volatile SvpwmHandler* svp)
{
    f32_t u1,u2,u3,tx,ty;
    int8_t cnt = 0;
    f32_t volAlpha,volBeta;
    volAlpha = svp->volAlphaBeta.com1;
    volBeta = svp->volAlphaBeta.com2;
    u1 = volBeta;
    u2 = sqrt3 / 2.f * volAlpha - 0.5f * volBeta;
    u3 = -sqrt3 / 2.f * volAlpha - 0.5f * volBeta;
    f32_t cons = sqrt3 * PerformanceCriticalTask_Period / svp->motorVoltage;

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
            ta = (PerformanceCriticalTask_Period - tx + ty) * 0.25f;
            tb = (PerformanceCriticalTask_Period + tx + ty) * 0.25f;
            tc = (PerformanceCriticalTask_Period - tx - ty) * 0.25f;
            break;
        case 2:
            svp->sector = 6;
            tx = cons * (sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ty = -cons * (volBeta);
            ta = (PerformanceCriticalTask_Period + tx + ty) * 0.25f;
            tb = (PerformanceCriticalTask_Period - tx - ty) * 0.25f;
            tc = (PerformanceCriticalTask_Period - tx + ty) * 0.25f;
            break;
        case 3:
            svp->sector = 1;
            tx = cons * (sqrt3 / 2.f * volAlpha - 0.5f * volBeta);
            ty = cons * (volBeta);
            ta = (PerformanceCriticalTask_Period + tx + ty) * 0.25f;
            tb = (PerformanceCriticalTask_Period - tx + ty) * 0.25f;
            tc = (PerformanceCriticalTask_Period - tx - ty) * 0.25f;
            break;
        case 4:
            svp->sector = 4;
            tx = -cons * (volBeta);
            ty = cons * (-sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ta = (PerformanceCriticalTask_Period - tx - ty) * 0.25f;
            tb = (PerformanceCriticalTask_Period - tx + ty) * 0.25f;
            tc = (PerformanceCriticalTask_Period + tx + ty) * 0.25f;
            break;
        case 5:
            svp->sector = 3;
            tx = cons * (volBeta);
            ty = -cons * (sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ta = (PerformanceCriticalTask_Period - tx - ty) * 0.25f;
            tb = (PerformanceCriticalTask_Period + tx + ty) * 0.25f;
            tc = (PerformanceCriticalTask_Period - tx + ty) * 0.25f;
            break;
        case 6:
            svp->sector = 5;
            tx = -cons * (sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ty = -cons * (-sqrt3 / 2.f * volAlpha + 0.5f * volBeta);
            ta = (PerformanceCriticalTask_Period - tx + ty) * 0.25f;
            tb = (PerformanceCriticalTask_Period - tx - ty) * 0.25f;
            tc = (PerformanceCriticalTask_Period + tx + ty) * 0.25f;
            break;
        default:
            ta = (PerformanceCriticalTask_Period) * 0.25f;
            tb = (PerformanceCriticalTask_Period) * 0.25f;
            tc = (PerformanceCriticalTask_Period) * 0.25f;
            break;
    }

    int32_t ccr1,ccr2,ccr3;
    
    ccr1 = ta * PerformanceCriticalTask_Timer_Frequency;
    ccr2 = tb * PerformanceCriticalTask_Timer_Frequency;
    ccr3 = tc * PerformanceCriticalTask_Timer_Frequency;

    if(ccr1 < 0) ccr1 = 0;
    else if(ccr1 > Timer_Period_ARR) ccr1 = Timer_Period_ARR;
    if(ccr2 < 0) ccr2 = 0;
    else if(ccr2 > Timer_Period_ARR) ccr2 = Timer_Period_ARR;
    if(ccr3 < 0) ccr3 = 0;
    else if(ccr3 > Timer_Period_ARR) ccr3 = Timer_Period_ARR;

    svp->ccr[0] = ccr1;
    svp->ccr[1] = ccr2;
    svp->ccr[2] = ccr3;
}

void SafetyTask(void)
{
    Components2 temp;
    static uint64_t sysRunTimeCnt = 0;
    switch(pSys->sysStu){
        case eWaitSysReset:
            Hardware_StopPWM();
            reset_All();
            pSys->sysStu = eWaitBusVoltage;
            break;
        case eWaitBusVoltage:
            pSens->busAndTemp.com1 = Hardware_GetBusVoltage() * 0.1f + pSens->busAndTemp.com1 * 0.9f;
            if(pSens->busAndTemp.com1 > MC_SafeVoltage * 0.85f && pSens->busAndTemp.com1 < MC_SafeVoltage * 1.15f){
                if(pSys->safeTaskTimeCnt++ > 50u){
                    pSys->safeTaskTimeCnt = 0;
                    pSys->sysStu = eWaitCapCharge;            
                }
            }
            else{pSys->safeTaskTimeCnt = 0;}
            break;
        case eWaitCapCharge:
            if(pSys->safeTaskTimeCnt++ > 50u){
                pSys->safeTaskTimeCnt = 0;
                Hardware_ForceSwitchOnAllLowSides();
                pSys->sysStu = eWaitCalADCOffset;
                break;
            }
            break;
        case eWaitCalADCOffset:
            temp = Hardware_GetCurrentOffset();
            pSens->currentOffset.com1 = 0.9f * pSens->currentOffset.com1 + 0.1f * temp.com1;
            pSens->currentOffset.com2 = 0.9f * pSens->currentOffset.com2 + 0.1f * temp.com2;
            if(pSys->safeTaskTimeCnt++ > 1000u){
                pSys->safeTaskTimeCnt = 0;
                Hardware_ResetAllLowSides();
                pSys->sysStu = eWaitMCStart;
                break;
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
        pSens->busAndTemp.com1 = Hardware_GetBusVoltage() * 0.99f + pSens->busAndTemp.com1 * 0.01f;
        if(pSens->busAndTemp.com1 < MC_SafeVoltage * 0.85f){
            if(pSys->safeTaskTimeCnt++ > 50u){
                Hardware_StopPWM();
                pSys->safeTaskTimeCnt = 0;
                pSys->sysError = eUnderVoltageError;
                pSys->sysStu = eWaitSysReset;
            }
        }
        else if(pSens->busAndTemp.com1 > MC_SafeVoltage * 1.15f){
            if(pSys->safeTaskTimeCnt++ > 50u){
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
    f32_t out;
    f32_t err = speedPIC->target - realSpeed;

    speedPIC->errInt += err * speedPIC->ts;

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
    Components2 out;
    f32_t err = idPIC->target - idqReal->com1;
    idPIC->errInt += err * idPIC->ts;

    if(idPIC->errInt > PIC_Current_Int_Limit) idPIC->errInt = PIC_Current_Int_Limit;
    else if(idPIC->errInt < -PIC_Current_Int_Limit) idPIC->errInt = -PIC_Current_Int_Limit;

    out.com1 = idPIC->errInt * PIC_Current_Ki + err * PIC_Current_Kp;

    if(out.com1 > PIC_Current_Out_Limit) out.com1 = PIC_Current_Out_Limit;
    else if(out.com1 < -PIC_Current_Out_Limit) out.com1 = -PIC_Current_Out_Limit;

    err = iqPIC->target - idqReal->com2;
    iqPIC->errInt += err * iqPIC->ts;

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
    pSens->currentAB.com1 -= pSens->currentOffset.com1;
    pSens->currentAB.com2 -= pSens->currentOffset.com2;
    if(pSys->sysStu == eSysRun){ 
        switch(pSys->controlMethod){
            case eMethod_AbsABZ:
                break;
            case eMethod_IncABZ:
                FOC_Method_IncABZ();
                break;
            case eMethod_ParIdentify:
                FOC_Method_ParIdentify();
                break;
            default:
                break;
        }
        pSVP->volAlphaBeta = Dq_AlphaBeta_Trans(&pSVP->volDQ,&pSens->sinCosVal);
        SectorCalModeSvpwm(pSVP);
        Hardware_PerformanceTaskEvent();
    }
    Hardware_SetCCR(pSVP->ccr[0],pSVP->ccr[1],pSVP->ccr[2]);
}

void FOC_Method_IncABZ(void)
{
    f32_t Ld,Lq,Flux,realEleSpeed;
    Components2 iAlphaBeta;
    Components2 piOut;
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
            else{pSys->focTaskTimeCnt = 0;pSys->focStep = eFOC_Step_2;}
            break;
        case eFOC_Step_2:
            if(pSys->focTaskTimeCnt++ < 5){
                pIncABZ->isABZEncoderAlignment = true;
                Hardware_SetIncABZEncoderTimCnt(1250);
                pIncABZ->lastEncoderCnt = 1250;
                reset_CurrentPICHandler();
            }
            else{pSys->focTaskTimeCnt = 0;pSys->focStep = eFOC_Step_3;}
            break;
        case eFOC_Step_3:
            pIdPIC->target = 0.f;
            pSys->focTaskTimeCnt = 0;
            Ld = pMotor->Ld;
            Lq = pMotor->Lq;
            Flux = pMotor->Flux;
            pSys->focStep = eFOC_Step_4;
            break;
        case eFOC_Step_4:
            realEleSpeed = Hardware_GetRealEleAngle();
            pSens->sinCosVal = Hardware_GetSinCosVal(realEleSpeed);
            iAlphaBeta = Abc_AlphaBeta_Trans(&pSens->currentAB);
            pSens->currentDQ = AlphaBeta_Dq_Trans(&iAlphaBeta,&pSens->sinCosVal);
            piOut = CurrentPIController(pIdPIC,pIqPIC,&pSens->currentDQ);
            piOut.com1 = piOut.com1 - pSens->currentDQ.com2 * Lq * realEleSpeed;
            piOut.com2 = piOut.com2 + (pSens->currentDQ.com1 * Ld + Flux) * realEleSpeed;
            pSVP->volDQ.com1 = piOut.com1;
            pSVP->volDQ.com2 = piOut.com2;
            pSys->focTaskTimeCnt = (pSys->focTaskTimeCnt % 10);
            if(pSys->focTaskTimeCnt++ == 0u){pIqPIC->target = SpeedPIController(pSpPIC,Hardware_GetEleSpeed());}
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
            if(pSys->focTaskTimeCnt++ == 0u){pSVP->volDQ.com1 = -pParmeterIndentify->injectSigAmp * polarity + 0.f;}
            MCParIdentify_Rs_Ls(pParmeterIndentify,pSens->currentDQ.com1);
            break;
        default:
            break;
    }
}



