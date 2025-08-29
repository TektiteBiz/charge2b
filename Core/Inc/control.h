#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "peripheral.h"

void ResetCurrent(bool chan1, float current);
bool ControlUpdate(bool batt1, float dT);

#endif /* __CONTROL_H */