#ifndef _MC_HVSENSORLESS_H_
#define _MC_HVSENSORLESS_H_

#include "mcVar.h"

extern void NonlinearFluxObsProcess(volatile MC_MotorPar* motor,volatile NonlinearFluxObsHandler* pNLFO,volatile PIC* sp,Components2* uAlphaBeta,Components2* iAlphaBeta);

#endif




