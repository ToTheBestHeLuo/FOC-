#ifndef _MC_SENSOR_H_
#define _MC_SENSOR_H_

#include "mcVar.h"

extern f32_t IncAbzCalculateRealEleAngle(volatile IncABZEncoder* pABZ);
extern f32_t IncAbzCalculateRealEleSpeed(volatile IncABZEncoder* pABZ,f32_t targetEleSpeed);

#endif


