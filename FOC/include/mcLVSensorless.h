#ifndef _MC_HVSENSORLESS_H_
#define _MC_HVSENSORLESS_H_

#include "mcVar.h"

extern f32_t HFPISensorlessObserver(volatile SensorHandler* sens,volatile HFPIHandler* hfpi);
extern f32_t HFSISensorlessObserver(volatile SensorHandler* sens,volatile HFSIHandler* hfsi);
extern f32_t HFSI_AngleCompensate(volatile HFSIHandler* hfsi);
extern bool NSIdentifyStateMachine(volatile NSIdentifyProcessHandler* ns,f32_t currentId);
extern bool NSCheckStateMachine(volatile NSCheckHandler* ns);

#endif
