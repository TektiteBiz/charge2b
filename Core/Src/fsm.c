#include "fsm.h"

#include "control.h"
#include "irmeas.h"
#include "peripheral.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

// Definitions
#define CHARGE_MODE_COUNT 4
const char* CHARGE_MODES[CHARGE_MODE_COUNT] = {"Slow", "Normal", "Fast",
                                               "Turbo"};
const float CHARGE_CURRENT[CHARGE_MODE_COUNT] = {0.8f, 1.2f, 1.6f, 2.0f};
const float TOPUP_CURRENT = 0.07f;
const float PRECHARGE_CURRENT = 0.2f;
#define MAX_TEMP 100.0f  // Celsius

float maxVoltage1 = 0.0f;
float maxVoltage2 = 0.0f;

// Charge state logic
CHARGESTATE state1 = CS_DISCONNECTED;
CHARGESTATE state2 = CS_DISCONNECTED;
uint32_t stateSetTime1 = 0;
uint32_t stateSetTime2 = 0;
uint8_t chargeMode1 = 0;
uint8_t chargeMode2 = 0;
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
void InitFSM() {
  ReadEEPROM(0x00, &chargeMode1, 1);
  ReadEEPROM(0x01, &chargeMode2, 1);

  // Check if needs fixing, not greater than -1 cuz if it is equal to charge
  // mode count that is internal resistance measurement
  if (chargeMode1 > CHARGE_MODE_COUNT) {
    chargeMode1 = 0;
    WriteEEPROM(0x00, &chargeMode1, 1);
  }
  if (chargeMode2 > CHARGE_MODE_COUNT) {
    chargeMode2 = 0;
    WriteEEPROM(0x01, &chargeMode2, 1);
  }
}
uint8_t getChargeMode(bool batt1) { return batt1 ? chargeMode1 : chargeMode2; }
void setChargeMode(uint8_t mode, bool batt1) {
  if (batt1) {
    chargeMode1 = mode;
    WriteEEPROM(0x00, &chargeMode1, 1);
  } else {
    chargeMode2 = mode;
    WriteEEPROM(0x01, &chargeMode2, 1);
  }
}
CHARGE_ERROR chargeError1 = CHARGE_NONE;
CHARGE_ERROR chargeError2 = CHARGE_NONE;
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
  setChargeError(CHARGE_NONE, batt1);

  // Check if battery is connected
  float voltage = BatteryVoltage(batt1);
  if (voltage >= 8.5f &&
      GetChargeStateTime(batt1) > 1000) {  // Battery connected
    if (getChargeMode(batt1) == CHARGE_MODE_COUNT) {
      SetChargeState(CS_IRMEAS, batt1);
    } else {
      SetChargeState(CS_CHARGE, batt1);
    }
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
    uint8_t newMode = getChargeMode(batt1) + 1;
    if (newMode >= CHARGE_MODE_COUNT + 1) {
      newMode = 0;
    }
    setChargeMode(newMode, batt1);
  }

  if (getChargeMode(batt1) == CHARGE_MODE_COUNT) {
    // IR measurement mode
    ResetIRMeas(batt1);
  } else {
    // Update charge mode current
    ResetCurrent(batt1, CHARGE_CURRENT[getChargeMode(batt1)]);
  }

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
  LEDWrite(batt1, 0.4f, 0.0f, 0.0f);

  // Charge battery
  ControlUpdate(batt1, dT);

  // Check if precharge needed
  if (BatteryVoltage(batt1) < 10.0f) {
    // Precharge needed
    ResetCurrent(batt1, PRECHARGE_CURRENT);
    SetChargeState(CS_PRECHARGE, batt1);
    return;
  }

  // 5s cooldown after plugging in battery
  if (GetChargeStateTime(batt1) < 5000) {
    return;
  }

  // Check if battery disconnected
  if (BatteryCurrent(batt1) < 0.1f) {
    // Battery disconnected
    SetChargeState(CS_DISCONNECTED, batt1);
    return;
  }

  // Check if overvoltage
  if (BatteryVoltage(batt1) > 14.3f) {
    // Overvoltage detected
    setChargeError(CHARGE_OVERVOLTAGE, batt1);
    SetChargeState(CS_ERROR, batt1);
    return;
  }

  // Check if overtime
  float tauMax = 3.0f / CHARGE_CURRENT[getChargeMode(batt1)] * 1.25f * 60.0f *
                 60.0f * 1000.0f;  // 3.0f comes from 3000mAh, 25% charge
                                   // efficiency factor, multiply
  // by 60 twice to convert hours to seconds
  if (GetChargeStateTime(batt1) > (uint32_t)tauMax) {
    setChargeError(CHARGE_OVERTIME, batt1);
    SetChargeState(CS_ERROR, batt1);
    return;
  }

  // Check if overtemperature
  if (ChargerTempCelsius(batt1) > MAX_TEMP) {
    SetChargeState(CS_OVERTEMP, batt1);
  }

  // Termination detection
  // First, make sure current is nominal
  float currErr =
      fabsf(CHARGE_CURRENT[getChargeMode(batt1)] - BatteryCurrent(batt1));
  if (currErr > 0.03f) {
    return;
  }

  // Maximum voltage recording
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
  // 70mV dV/dt
  if (maxVolt - BatteryVoltage(batt1) > 0.07f) {
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
      SetChargeState(CS_TOPUP, batt1);
      return;
    }
  }
}

void fsm_OVERTEMP(bool batt1) {
  // Writes
  LEDWrite(batt1, 0.15f, 0.15f, 0.0f);

  // Control
  ResetCurrent(batt1, CHARGE_CURRENT[getChargeMode(batt1)]);
  EnableReg(batt1, false);  // Disable regulator

  // Check if battery disconnected
  if (BatteryVoltage(batt1) < 8.5f) {
    // Battery disconnected
    SetChargeState(CS_DISCONNECTED, batt1);
  }

  // Check if temperature ok
  if (ChargerTempCelsius(batt1) < MAX_TEMP - 10.0f) {
    // Temperature ok, go back to charge state
    SetChargeState(CS_CHARGE, batt1);
    return;
  }
}

void fsm_TOPUP(bool batt1, float dT) {
  // Writes
  LEDWrite(batt1, 0.0f, 0.4f, 0.0f);
  ControlUpdate(batt1, dT);

  if (GetChargeStateTime(batt1) < 5000) {
    return;
  }

  // Check if battery disconnected
  if (BatteryCurrent(batt1) < 0.03f) {
    // Battery disconnected
    SetChargeState(CS_DISCONNECTED, batt1);
    return;
  }

  // Check if overvoltage
  if (BatteryVoltage(batt1) > 14.4f) {
    // Overvoltage detected
    setChargeError(CHARGE_OVERVOLTAGE, batt1);
    SetChargeState(CS_ERROR, batt1);
    return;
  }

  // Check if overtime (1 hour)
  if (GetChargeStateTime(batt1) > 60.0f * 60.0f * 1000.0f) {
    SetChargeState(CS_DONE, batt1);
    return;
  }
}

void fsm_IRMEAS(bool batt1, float dT) {
  // Writes
  LEDWrite(batt1, 0.2f, 0.0f, 0.1f);

  // Let orig voltage measurement settle
  if (GetChargeStateTime(batt1) < 1500) {
    EnableReg(batt1, false);
    return;
  }

  // Check if battery disconnected
  if (IRMeasDone(batt1) || GetChargeStateTime(batt1) < 7500) {
    // Battery isn't powered or getting up to current
    if (BatteryVoltage(batt1) < 8.5f) {  // Battery disconnected
      SetChargeState(CS_DISCONNECTED, batt1);
      return;
    }
  } else {
    if (BatteryCurrent(batt1) < 0.03f) {
      // Battery disconnected
      SetChargeState(CS_DISCONNECTED, batt1);
      return;
    }
  }

  // Check if overvoltage
  if (BatteryVoltage(batt1) > 14.3f) {
    // Overvoltage detected
    setChargeError(CHARGE_OVERVOLTAGE, batt1);
    SetChargeState(CS_ERROR, batt1);
    return;
  }

  // Update IRMeas
  IRMeasUpdate(batt1, dT);
}

void fsm_DONE(bool batt1) {
  // Writes
  LEDWrite(batt1, 0.0f, 0.4f, 0.0f);
  EnableReg(batt1, false);
  setChargeError(CHARGE_NONE, batt1);
  // Check if battery disconnected
  if (BatteryVoltage(batt1) < 8.5f) {  // Battery disconnected
    SetChargeState(CS_DISCONNECTED, batt1);
    return;
  }
}

void fsm_PRECHARGE(bool batt1, float dT) {
  // Writes
  LEDWrite(batt1, 0.4f, 0.0f, 0.0f);
  ControlUpdate(batt1, dT);

  if (GetChargeStateTime(batt1) < 5000) {
    return;
  }

  // Check if battery disconnected
  if (BatteryCurrent(batt1) < 0.1f) {
    // Battery disconnected
    SetChargeState(CS_DISCONNECTED, batt1);
    return;
  }

  // Check if done with precharge
  if (BatteryVoltage(batt1) > 10.5f) {
    ResetCurrent(batt1, CHARGE_CURRENT[getChargeMode(batt1)]);
    SetChargeState(CS_CHARGE, batt1);
    return;
  }

  // Check if overtime (4 hours)
  if (GetChargeStateTime(batt1) > 4.0f * 60.0f * 60.0f * 1000.0f) {
    // Precharge overtime
    setChargeError(PRECHARGE_OVERTIME, batt1);
    SetChargeState(CS_ERROR, batt1);
    return;
  }
}

void fsm_Error(bool batt1) {
  // Writes
  LEDWrite(batt1, 0.2f, 0.1f, 0.0f);
  EnableReg(batt1, false);

  if (GetChargeStateTime(batt1) < 5000) {
    return;
  }

  // Check if battery disconnected
  if (BatteryVoltage(batt1) < 8.5f) {  // Battery disconnected
    SetChargeState(CS_DISCONNECTED, batt1);
    return;
  }
}

// Main FSM function
void fsm_Run(bool batt1, float dT) {
  switch (batt1 ? state1 : state2) {
    case CS_DISCONNECTED:
      fsm_DISCONNECTED(batt1);
      break;
    case CS_CHARGE:
      fsm_CHARGE(batt1, dT);
      break;
    case CS_TOPUP:
      fsm_TOPUP(batt1, dT);
      break;
    case CS_DONE:
      fsm_DONE(batt1);
      break;
    case CS_PRECHARGE:
      fsm_PRECHARGE(batt1, dT);
      break;
    case CS_OVERTEMP:
      fsm_OVERTEMP(batt1);
      break;
    case CS_IRMEAS:
      fsm_IRMEAS(batt1, dT);
      break;
    case CS_ERROR:
      fsm_Error(batt1);
      break;
  }
}

// UI
#include <stdarg.h>
void ssd1306_PrintLine(int row, const char* fmt, ...) {
  char buf[32];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  // 10px font height, add 2px spacing for readability
  int y = row * 12;
  ssd1306_SetCursor(0, y);
  ssd1306_WriteString(buf, Font_7x10, White);
}

void fsm_Render(bool batt1) {
  ssd1306_Display(batt1);
  ssd1306_Fill(Black);
  switch (batt1 ? state1 : state2) {
    case CS_DISCONNECTED:
      ssd1306_PrintLine(0, "Idle");
      ssd1306_PrintLine(2, "Mode:");
      uint8_t mode = getChargeMode(batt1);
      if (mode == CHARGE_MODE_COUNT) {
        ssd1306_PrintLine(3, "IR Measurement");
      } else {
        ssd1306_PrintLine(3, "%s - %.1fA", CHARGE_MODES[mode],
                          CHARGE_CURRENT[mode]);
      }
      break;
    case CS_CHARGE:
      ssd1306_PrintLine(0, "Battery Charging");
      float voltage = BatteryVoltage(batt1);
      float current = BatteryCurrent(batt1);
      ssd1306_PrintLine(1, "Voltage: %.1fV", voltage);
      ssd1306_PrintLine(2, "Current: %.1fA", current);
      ssd1306_PrintLine(3, "Power: %.1fW",
                        voltage * current);  // Power in Watts
      ssd1306_PrintLine(4, "%02lu:%02lu | %.1fC",
                        GetChargeStateTime(batt1) / 60000,
                        (GetChargeStateTime(batt1) % 60000) / 1000,
                        ChargerTempCelsius(batt1));
      break;
    case CS_TOPUP:
      ssd1306_PrintLine(0, "Trickle Charging");
      ssd1306_PrintLine(1, "Voltage: %.1fV", BatteryVoltage(batt1));
      ssd1306_PrintLine(2, "Current: %.2fA", BatteryCurrent(batt1));
      ssd1306_PrintLine(3, "Power: %.2fW",
                        BatteryVoltage(batt1) * BatteryCurrent(batt1));
      ssd1306_PrintLine(4, "Time: %02lu:%02lu",
                        GetChargeStateTime(batt1) / 60000,
                        (GetChargeStateTime(batt1) % 60000) / 1000);
      break;
    case CS_OVERTEMP:
      ssd1306_PrintLine(0, "Overtemperature");
      ssd1306_PrintLine(1, "Voltage: %.1fV", BatteryVoltage(batt1));
      ssd1306_PrintLine(2, "Temperature: %.1fC", ChargerTempCelsius(batt1));
      ssd1306_PrintLine(3, "Time: %02lu:%02lu",
                        GetChargeStateTime(batt1) / 60000,
                        (GetChargeStateTime(batt1) % 60000) / 1000);
      break;
    case CS_DONE:
      ssd1306_PrintLine(0, "Charging Complete");
      ssd1306_PrintLine(1, "Voltage: %.1fV", BatteryVoltage(batt1));
      break;
    case CS_PRECHARGE:
      ssd1306_PrintLine(0, "Pre-charging");
      ssd1306_PrintLine(1, "Voltage: %.1fV", BatteryVoltage(batt1));
      ssd1306_PrintLine(2, "Current: %.1fA", BatteryCurrent(batt1));
      ssd1306_PrintLine(3, "Power: %.1fW",
                        BatteryVoltage(batt1) * BatteryCurrent(batt1));
      ssd1306_PrintLine(4, "Time: %02lu:%02lu",
                        GetChargeStateTime(batt1) / 60000,
                        (GetChargeStateTime(batt1) % 60000) / 1000);
      break;
    case CS_IRMEAS:
      ssd1306_PrintLine(0, "IR Measurement");
      ssd1306_PrintLine(1, "%.1fmOhm", GetIRValue(batt1));
      ssd1306_PrintLine(2, "Voltage: %.1fV", BatteryVoltage(batt1));
      if (!IRMeasDone(batt1)) {
        ssd1306_PrintLine(3, "Current: %.1fA", BatteryCurrent(batt1));
        ssd1306_PrintLine(4, "Time: %02lu:%02lu",
                          GetChargeStateTime(batt1) / 60000,
                          (GetChargeStateTime(batt1) % 60000) / 1000);
      } else {
        ssd1306_PrintLine(3, "Done");
      }
      break;
    case CS_ERROR:
      ssd1306_PrintLine(0, "Charging Error");
      switch (getChargeError(batt1)) {
        case CHARGE_NONE:
          ssd1306_PrintLine(1, "No Error");
          break;
        case CHARGE_OVERTIME:
          ssd1306_PrintLine(1, "Charge Overtime");
          break;
        case PRECHARGE_OVERTIME:
          ssd1306_PrintLine(1, "Precharge Overtime");
          break;
        case CHARGE_OVERVOLTAGE:
          ssd1306_PrintLine(1, "Overvoltage");
          break;
      }
      ssd1306_PrintLine(2, "Voltage: %.1fV", BatteryVoltage(batt1));
      break;
  }
  ssd1306_UpdateScreen();
}