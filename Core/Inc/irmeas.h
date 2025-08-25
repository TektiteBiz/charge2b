#ifndef __IRMEAS_H
#define __IRMEAS_H

#include "peripheral.h"

float GetIRValue(bool batt1);
void IRMeasUpdate(bool batt1, float dT);
void ResetIRMeas(bool batt1);
bool IRMeasDone(bool batt1);

#endif /* __IRMEAS_H */