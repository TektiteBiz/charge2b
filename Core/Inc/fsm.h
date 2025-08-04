#ifndef __FSM_H
#define __FSM_H

#include "main.h"

typedef enum CHARGESTATE {
  DISCONNECTED,
  PRECHARGE,
  CHARGE,
  TOPUP,
  DONE,

  ERROR,
} CHARGESTATE;

typedef enum CHARGE_ERROR {
  NONE,
  CHARGE_OVERTIME,
  PRECHARGE_OVERTIME,
  CHARGE_OVERVOLTAGE,
} CHARGE_ERROR;

void SetChargeState(CHARGESTATE newState, bool batt1);
uint32_t GetChargeStateTime(bool batt1);
void fsm_Run(bool batt1, float dT);

#define CHARGE_MODE_COUNT 4
const char* CHARGE_MODES[CHARGE_MODE_COUNT] = {"Slow", "Normal", "Fast",
                                               "Turbo"};
const float CHARGE_CURRENT[CHARGE_MODE_COUNT] = {0.8f, 1.2f, 1.6f, 2.0f};
const float TOPUP_CURRENT = 0.07f;
const float PRECHARGE_CURRENT = 0.2f;

#endif /* __FSM_H */