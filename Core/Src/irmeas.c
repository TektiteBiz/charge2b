#include "irmeas.h"

#define SETPOINT_COUNT 7
const float irSetpoint[SETPOINT_COUNT] = {
    1.2f, 1.6f, 0.8f, 2.0f,
    0.4f, 1.6f, 0.8f};  // First setpoint is just to figure out orig voltage

uint32_t setpointHitTime1 = 0;
uint32_t setpointHitTime2 = 0;

int setpointInd1 = 0;
int setpointInd2 = 0;

float irVal1 = 0.0f;
float irVal2 = 0.0f;

float voltage1 = 8.5f;
float voltage2 = 8.5f;

float origVoltage1 = 0.0f;
float origVoltage2 = 0.0f;

#define SETPOINT_KI 0.4f
#define kI_0A 0.18f  // Integrator gain when low battery current (battery likely
// disconnected, its possible that its just initial connection though)

void ResetIRMeas(bool batt1) {
  if (batt1) {
    setpointHitTime1 = 0;
    setpointInd1 = 0;
    voltage1 = 8.5f;
    irVal1 = 0.0f;
    origVoltage1 = 0.0f;
  } else {
    setpointHitTime2 = 0;
    setpointInd2 = 0;
    voltage2 = 8.5f;
    irVal2 = 0.0f;
    origVoltage2 = 0.0f;
  }
}

void IRMeasUpdate(bool batt1, float dT) {
  // Check if done
  int setpointInd = batt1 ? setpointInd1 : setpointInd2;
  if (setpointInd >= SETPOINT_COUNT) {
    EnableReg(batt1, false);
    return;
  }

  // Check if need to init origVoltage
  float origVoltage = batt1 ? origVoltage1 : origVoltage2;

  // Controller update for setpoint
  float voltage = BatteryVoltage(batt1);
  float currApplVolt = batt1 ? voltage1 : voltage2;
  if (voltage - currApplVolt > 0.5f) {
    currApplVolt = voltage;
  } else {
    float irSet = irSetpoint[setpointInd];
    float battCurr = BatteryCurrent(batt1);
    float err = irSet - battCurr;
    if (battCurr < 0.04f) {
      currApplVolt += kI_0A * err * dT;
    } else {
      currApplVolt += SETPOINT_KI * err * dT;
    }

    if (currApplVolt > 15.7f) {
      currApplVolt = 15.7f;
    }
  }
  if (batt1) {
    voltage1 = currApplVolt;
  } else {
    voltage2 = currApplVolt;
  }
  WriteVoltage(batt1, currApplVolt);

  // Check if time to move on to next setpoint
  uint32_t setpointHitTime = batt1 ? setpointHitTime1 : setpointHitTime2;
  if (setpointHitTime > 0) {
    // See if we've been at the setpoint for 1s
    if (HAL_GetTick() - setpointHitTime >= 1000) {
      // Save measurement
      if (setpointInd == 0) {
        // First setpoint, save original voltage
        if (batt1) {
          origVoltage1 = voltage;
        } else {
          origVoltage2 = voltage;
        }
      } else {
        float irValIncr =
            (voltage - origVoltage) / (BatteryCurrent(batt1) - irSetpoint[0]);
        if (batt1) {
          irVal1 += irValIncr;
        } else {
          irVal2 += irValIncr;
        }
      }

      // Move to next setpoint
      setpointInd++;
      if (batt1) {
        setpointInd1 = setpointInd;
        setpointHitTime1 = 0;
      } else {
        setpointInd2 = setpointInd;
        setpointHitTime2 = 0;
      }
    }
    return;
  }

  // See if we've hit setpoint
  if (fabsf(BatteryCurrent(batt1) - irSetpoint[setpointInd]) < 0.03f) {
    if (batt1) {
      setpointHitTime1 = HAL_GetTick();
    } else {
      setpointHitTime2 = HAL_GetTick();
    }
  }
}

float GetIRValue(bool batt1) {
  if ((batt1 ? setpointInd1 : setpointInd2) < 2) {
    return 0.0f;
  }
  return (batt1 ? irVal1 / (setpointInd1 - 1) : irVal2 / (setpointInd2 - 1)) *
         1000.0f;
}

bool IRMeasDone(bool batt1) {
  return (batt1 ? setpointInd1 : setpointInd2) >= SETPOINT_COUNT;
}