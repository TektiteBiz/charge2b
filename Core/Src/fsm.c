#include "fsm.h"

#include "control.h"
#include "peripheral.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

float maxVoltage1 = 0.0f;
float maxVoltage2 = 0.0f;

// Charge state logic
CHARGESTATE state1 = DISCONNECTED;
CHARGESTATE state2 = DISCONNECTED;
uint32_t stateSetTime1 = 0;
uint32_t stateSetTime2 = 0;
int chargeMode1 = 0;
int chargeMode2 = 0;
void SetChargeState(CHARGESTATE newState, bool batt1) {
  if (batt1) {
    state1 = newState;
    stateSetTime1 = HAL_GetTick();
  } else {
    state2 = newState;
    stateSetTime2 = HAL_GetTick();
  }
}
uint32_t GetChargeStateTime(bool batt1) {
  if (batt1) {
    return HAL_GetTick() - stateSetTime1;
  } else {
    return HAL_GetTick() - stateSetTime2;
  }
}
int getChargeMode(bool batt1) { return batt1 ? chargeMode1 : chargeMode2; }
void setChargeMode(int mode, bool batt1) {
  if (batt1) {
    chargeMode1 = mode;
  } else {
    chargeMode2 = mode;
  }
}
CHARGE_ERROR chargeError1 = NONE;
CHARGE_ERROR chargeError2 = NONE;
void setChargeError(CHARGE_ERROR error, bool batt1) {
  if (batt1) {
    chargeError1 = error;
  } else {
    chargeError2 = error;
  }
}
CHARGE_ERROR getChargeError(bool batt1) {
  return batt1 ? chargeError1 : chargeError2;
}

// Handlers
bool modePressed1 = false;
bool modePressed2 = false;
void fsm_DISCONNECTED(bool batt1) {
  // Writes
  LEDWrite(batt1, 0.0f, 0.0f, 0.0f);
  EnableReg(batt1, false);
  setChargeError(NONE, batt1);

  // Check if battery is connected
  float voltage = batteryVoltage(batt1);
  if (voltage >= 8.5f &&
      GetChargeStateTime(batt1) > 1000) {  // Battery connected
    SetChargeState(CHARGE, batt1);
  }

  // Handle mode button press, switch modes
  if (!(batt1 ? modePressed1 : modePressed2) &&
      HAL_GPIO_ReadPin(batt1 ? MODE1_GPIO_Port : MODE2_GPIO_Port,
                       batt1 ? MODE1_Pin : MODE2_Pin) == GPIO_PIN_RESET) {
    if (batt1) {
      modePressed1 = true;
    } else {
      modePressed2 = true;
    }
  }
  if ((batt1 ? modePressed1 : modePressed2) &&
      HAL_GPIO_ReadPin(batt1 ? MODE1_GPIO_Port : MODE2_GPIO_Port,
                       batt1 ? MODE1_Pin : MODE2_Pin) == GPIO_PIN_SET) {
    if (batt1) {
      modePressed1 = false;
    } else {
      modePressed2 = false;
    }
    // Change charge mode
    int newMode = getChargeMode(batt1) + 1;
    if (newMode >= CHARGE_MODE_COUNT) {
      newMode = 0;
    }
    setChargeMode(newMode, batt1);
  }
  // Update charge mode current
  ResetCurrent(batt1, CHARGE_CURRENT[getChargeMode(batt1)]);

  if (batt1) {
    maxVoltage1 = 0.0f;
  } else {
    maxVoltage2 = 0.0f;
  }
}

uint32_t voltageDropTime1 = 0;
uint32_t voltageDropTime2 = 0;
void fsm_CHARGE(bool batt1, float dT) {
  // Writes
  LEDWrite(batt1, 0.2f, 0.0f, 0.0f);

  // Charge battery
  ControlUpdate(batt1, dT);

  // Check if precharge needed
  if (BatteryVoltage(batt1) < 10.0f) {
    // Precharge needed
    ResetCurrent(batt1, PRECHARGE_CURRENT);
    SetChargeState(PRECHARGE, batt1);
    return;
  }

  // Termination detection
  if (GetChargeStateTime(batt1) < 1000) {
    return;
  }
  float maxVolt = batt1 ? maxVoltage1 : maxVoltage2;
  if (BatteryVoltage(batt1) > maxVolt) {
    if (batt1) {
      maxVoltage1 = BatteryVoltage(batt1);
      voltageDropTime1 = 0;
    } else {
      maxVoltage2 = BatteryVoltage(batt1);
      voltageDropTime2 = 0;
    }
  }
  // 40mV dV/dt
  if (maxVolt - BatteryVoltage(batt1) > 0.04f) {
    uint32_t voltageDropTime = batt1 ? voltageDropTime1 : voltageDropTime2;
    if (voltageDropTime == 0) {  // Set voltage drop time
      if (batt1) {
        voltageDropTime1 = HAL_GetTick();
      } else {
        voltageDropTime2 = HAL_GetTick();
      }
    }

    // Check if voltage drop time exceeded
    if (HAL_GetTick() - voltageDropTime > 2000) {
      // Go to top-up phase
      ResetCurrent(batt1, TOPUP_CURRENT);
      SetChargeState(TOPUP, batt1);
      return;
    }
  }

  // Check if battery disconnected
  if (BatteryCurrent(batt1) < 0.1f) {
    // Battery disconnected
    SetChargeState(DISCONNECTED, batt1);
    return;
  }

  // Check if overvoltage
  if (BatteryVoltage(batt1) > 14.9f) {
    // Overvoltage detected
    setChargeError(CHARGE_OVERVOLTAGE, batt1);
    SetChargeState(ERROR, batt1);
    return;
  }

  // Check if overtime
  float tauMax =
      3.0f / CHARGE_CURRENT[getChargeMode(batt1)] * 1.25f * 60.0f *
      60.0f;  // 3.0f comes from 3000mAh, 25% charge efficiency factor, multiply
  // by 60 twice to convert hours to seconds
  if (GetChargeStateTime(batt1) > tauMax) {
    setChargeError(CHARGE_OVERTIME, batt1);
    SetChargeState(ERROR, batt1);
    return;
  }
}

void fsm_TOPUP(bool batt1, float dT) {
  // Writes
  LEDWrite(batt1, 0.0f, 0.2f, 0.0f);
  ControlUpdate(batt1, dT);

  if (GetChargeStateTime(batt1) < 1000) {
    return;
  }

  // Check if battery disconnected
  if (BatteryCurrent(batt1) < 0.03f) {
    // Battery disconnected
    SetChargeState(DISCONNECTED, batt1);
    return;
  }

  // Check if overvoltage
  if (BatteryVoltage(batt1) > 14.9f) {
    // Overvoltage detected
    setChargeError(CHARGE_OVERVOLTAGE, batt1);
    SetChargeState(ERROR, batt1);
    return;
  }

  // Check if overtime (1 hour)
  if (GetChargeStateTime(batt1) > 60.0f * 60.0f) {
    SetChargeState(DONE, batt1);
    return;
  }
}

void fsm_DONE(bool batt1) {
  // Writes
  LEDWrite(batt1, 0.0f, 0.2f, 0.0f);
  EnableReg(batt1, false);
  setChargeError(NONE, batt1);
  // Check if battery disconnected
  if (BatteryVoltage(batt1) < 8.5f) {  // Battery disconnected
    SetChargeState(DISCONNECTED, batt1);
    return;
  }
}

void fsm_PRECHARGE(bool batt1, float dT) {
  // Writes
  LEDWrite(batt1, 0.2f, 0.0f, 0.0f);
  ControlUpdate(batt1, dT);

  if (GetChargeStateTime(batt1) < 1000) {
    return;
  }

  // Check if battery disconnected
  if (BatteryCurrent(batt1) < 0.1f) {
    // Battery disconnected
    SetChargeState(DISCONNECTED, batt1);
    return;
  }

  // Check if done with precharge
  if (BatteryVoltage(batt1) > 10.5f) {
    ResetCurrent(batt1, CHARGE_CURRENT[getChargeMode(batt1)]);
    SetChargeState(CHARGE, batt1);
    return;
  }

  // Check if overtime (4 hours)
  if (GetChargeStateTime(batt1) > 4.0f * 60.0f * 60.0f) {
    // Precharge overtime
    setChargeError(PRECHARGE_OVERTIME, batt1);
    SetChargeState(ERROR, batt1);
    return;
  }
}

void fsm_Error(bool batt1) {
  // Writes
  LEDWrite(batt1, 0.15f, 0.15f, 0.0f);
  EnableReg(batt1, false);

  if (GetChargeStateTime(batt1) < 1000) {
    return;
  }

  // Check if battery disconnected
  if (BatteryVoltage(batt1) < 8.5f) {  // Battery disconnected
    SetChargeState(DISCONNECTED, batt1);
    return;
  }
}

// Main FSM function
void fsm_Run(bool batt1, float dT) {
  switch (batt1 ? state1 : state2) {
    case DISCONNECTED:
      fsm_DISCONNECTED(batt1);
      break;
    case CHARGE:
      fsm_CHARGE(batt1, dT);
      break;
    case TOPUP:
      fsm_TOPUP(batt1, dT);
      break;
    case DONE:
      fsm_DONE(batt1);
      break;
    case PRECHARGE:
      fsm_PRECHARGE(batt1, dT);
      break;
    case ERROR:
      fsm_Error(batt1);
      break;
  }
}