#include "fsm.h"

#include "peripheral.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

float maxVoltage[2] = {0.0f, 0.0f};

// Charge state logic
CHARGESTATE state1 = DISCONNECTED;
CHARGESTATE state2 = DISCONNECTED;
uint32_t stateSetTime1 = 0;
uint32_t stateSetTime2 = 0;
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
// Handlers
void fsm_DISCONNECTED(bool batt1) {
  // Writes
  // WriteCurrent(batt1, 0.0f);
  LEDWrite(batt1, 0.0f, 0.0f, 0.0f);

  // Display battery disconnected on ssd1306
  ssd1306_Display(batt1);
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Battery Disconnected", Font_6x8, White);
  ssd1306_UpdateScreen();

  // Read battery voltage
  float voltage = batteryVoltage(batt1);
  if (voltage < 15.0f) {  // Battery connected
    SetChargeState(CHARGE, batt1);
  }

  if (batt1) {
    maxVoltage[0] = 0.0f;
  } else {
    maxVoltage[1] = 0.0f;
  }
}

void fsm_CHARGE(bool batt1) {
  // Writes
  // WriteCurrent(batt1, 2.0f);
  LEDWrite(batt1, 0.2f, 0.0f, 0.0f);

  // Display battery charging on ssd1306
  ssd1306_Display(batt1);
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Battery Charging", Font_6x8, White);
  // Display voltage, current, and power
  float voltage = batteryVoltage(batt1);
  float current = 2.0f;  // Assume constant current for simplicity
  float power = voltage * current;
  ssd1306_SetCursor(0, 10);
  ssd1306_WriteString("Voltage: ", Font_6x8, White);
  ssd1306_SetCursor(60, 10);
  char v_buf[12];
  snprintf(v_buf, sizeof(v_buf), "%.2f", voltage);
  ssd1306_WriteString(v_buf, Font_6x8, White);
  ssd1306_SetCursor(0, 20);
  ssd1306_WriteString("Current: ", Font_6x8, White);
  ssd1306_SetCursor(60, 20);
  char c_buf[12];
  snprintf(c_buf, sizeof(c_buf), "%.2f", current);
  ssd1306_WriteString(c_buf, Font_6x8, White);
  ssd1306_SetCursor(0, 30);
  ssd1306_WriteString("Power: ", Font_6x8, White);
  ssd1306_SetCursor(60, 30);
  char p_buf[12];
  snprintf(p_buf, sizeof(p_buf), "%.2f", power);
  ssd1306_WriteString(p_buf, Font_6x8, White);
  ssd1306_SetCursor(0, 40);
  ssd1306_UpdateScreen();

  // Read battery voltage & update state (only do 1s after charge started, for
  // stability)
  if (GetChargeStateTime(batt1) > 1000) {
    if (voltage > maxVoltage[batt1 ? 0 : 1]) {  // New max voltage
      maxVoltage[batt1 ? 0 : 1] = voltage;
    }
    if (voltage < maxVoltage[batt1 ? 0 : 1] -
                      0.1f) {  // Fully charged - delta V/dt <100mV
      SetChargeState(CHARGE_DONE, batt1);
    }
    if (voltage > 15.0f) {  // Disconnected
      SetChargeState(DISCONNECTED, batt1);
    }
  }

  // TODO: Safety features
}

void fsm_CHARGE_DONE(bool batt1) {
  // Writes
  // WriteCurrent(batt1, 0.0f);
  LEDWrite(batt1, 0.0f, 0.2f, 0.0f);

  // Display battery charged on ssd1306
  ssd1306_Display(batt1);
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Battery Charged", Font_6x8, White);
  // Display voltage
  float voltage = batteryVoltage(batt1);
  ssd1306_SetCursor(0, 10);
  ssd1306_WriteString("Voltage: ", Font_6x8, White);
  ssd1306_SetCursor(60, 10);
  char v_buf[12];
  snprintf(v_buf, sizeof(v_buf), "%.2f", voltage);
  ssd1306_WriteString(v_buf, Font_6x8, White);
  ssd1306_UpdateScreen();

  // Read battery voltage
  if (voltage > 15.0f) {  // Battery connected
    SetChargeState(DISCONNECTED, batt1);
  }
}

// Main FSM function
void fsm_Run(bool batt1) {
  switch (batt1 ? state1 : state2) {
    case DISCONNECTED:
      fsm_DISCONNECTED(batt1);
      break;
    case CHARGE:
      fsm_CHARGE(batt1);
      break;
    case CHARGE_DONE:
      fsm_CHARGE_DONE(batt1);
      break;
    default:
      // Handle other states if needed
      printf("Unknown state: %d\n", batt1 ? state1 : state2);
      // WriteCurrent(batt1, 0.0f);
      LEDWrite(batt1, 0.0f, 0.0f, 0.2f);
      break;
  }
}