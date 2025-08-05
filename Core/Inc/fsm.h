#ifndef __FSM_H
#define __FSM_H

#include "main.h"

typedef enum CHARGESTATE {
  CS_DISCONNECTED,
  CS_PRECHARGE,
  CS_CHARGE,
  CS_TOPUP,
  CS_DONE,
  CS_ERROR,
} CHARGESTATE;

typedef enum CHARGE_ERROR {
  CHARGE_NONE,
  CHARGE_OVERTIME,
  PRECHARGE_OVERTIME,
  CHARGE_OVERVOLTAGE,
} CHARGE_ERROR;

void SetChargeState(CHARGESTATE newState, bool batt1);
uint32_t GetChargeStateTime(bool batt1);
void fsm_Run(bool batt1, float dT);
void fsm_Render(bool batt1);
void InitFSM();

#endif /* __FSM_H */