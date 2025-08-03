#include "control.h"

// Integrator variables
float m_i1 = 8.5f;
float m_i2 = 8.5f;

float curr1 = 0.0f;
float curr2 = 0.0f;

#define kI 0.4f  // Integrator gain

void ResetCurrent(bool chan1, float current) {
  if (chan1) {
    curr1 = current;
    m_i1 = 8.5f;  // Reset integrator for channel 1
  } else {
    curr2 = current;
    m_i2 = 8.5f;  // Reset integrator for channel 2
  }
}
void ControlUpdate(bool batt1, float dT) {
  float voltage = BatteryVoltage(batt1);
  float iNew = batt1 ? m_i1 : m_i2;
  if ((voltage - iNew) > 0.5f) {  // Battery voltage higher than integrator
    iNew = voltage;
  } else {
    float err = (batt1 ? curr1 : curr2) - BatteryCurrent(batt1);
    iNew += kI * err * dT;  // Integrate error
  }
  WriteVoltage(batt1, iNew);
  if (batt1) {
    m_i1 = iNew;
  } else {
    m_i2 = iNew;
  }
}