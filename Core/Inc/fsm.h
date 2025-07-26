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
void filterReset(bool batt1);

#endif /* __FSM_H */