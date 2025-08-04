#ifndef __FSM_H
#define __FSM_H

#include "main.h"

typedef enum CHARGESTATE {
  DISCONNECTED,
  PRECHARGE,
  PRECHARGE_OVERTIME,
  CHARGE,
  CHARGE_OVERTIME,
  CHARGE_OVERVOLTAGE,
  UNDERVOLTAGE,
  CHARGE_DONE,
} CHARGESTATE;

void SetChargeState(CHARGESTATE newState, bool batt1);
uint32_t GetChargeStateTime(bool batt1);
void fsm_Run(bool batt1);

#define CHARGE_MODE_COUNT 4
const char* CHARGE_MODES[CHARGE_MODE_COUNT] = {"Slow", "Normal", "Fast",
                                               "Turbo"};
const float CHARGE_CURRENT[CHARGE_MODE_COUNT] = {1.0f, 1.3f, 1.7f, 2.0f};
const float TOPUP_CURRENT = 0.07f;

#endif /* __FSM_H */