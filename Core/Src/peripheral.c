#include "peripheral.h"

#include "stusb_registers.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

void LEDWrite(bool LED1, float r, float g, float b) {
  TIM_HandleTypeDef* htim = LED1 ? &htim1 : &htim3;
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(r * 65535.0f));
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(g * 65535.0f));
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(b * 65535.0f));
}

extern I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef USB_Write_Raw(uint8_t reg, uint8_t* dataW, uint16_t len) {
  uint8_t txBuffer[len + 1];
  txBuffer[0] = reg;
  if (len > 0) {
    memcpy(&txBuffer[1], dataW, len);
  }
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
      &hi2c1, USB_ADDRESS, txBuffer, len + 1, HAL_MAX_DELAY);
  return status;
}

HAL_StatusTypeDef USB_Read_Raw(uint8_t reg, uint8_t* dataR, uint16_t len) {
  HAL_StatusTypeDef status;
  status = HAL_I2C_Master_Transmit(&hi2c1, USB_ADDRESS, &reg, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return status;
  }
  return HAL_I2C_Master_Receive(&hi2c1, USB_ADDRESS, dataR, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef USB_PDONumber(uint8_t* num) {
  HAL_StatusTypeDef status = USB_Read_Raw(USB_DPM_PDO_NUMB, num, 1);
  *num &= 0x07;
  return status;
}

// 0 for PDO 1, 1 for PDO 2, 2 for PDO 3
HAL_StatusTypeDef USB_ReadPDO(uint8_t pdo_num, float* voltage, float* current) {
  uint8_t buf[4];
  uint32_t pdoData = 0;

  // PDO1:0x85, PDO2:0x89, PDO3:0x8D
  HAL_StatusTypeDef status = USB_Read_Raw(0x85 + (pdo_num * 4), buf, 4);
  if (status != HAL_OK) {
    return status;
  }

  // Combine into pdoData
  for (uint8_t i = 0; i < 4; i++) {
    uint32_t tempData = buf[i];
    tempData = (tempData << (i * 8));
    pdoData += tempData;
  }

  // Get voltage
  uint32_t v_data = (pdoData >> 10) & 0x3FF;
  *voltage = ((float)v_data) / 20.0;

  // Get current
  uint32_t c_data = pdoData & 0x3FF;
  *current = ((float)c_data) * 0.01;

  return status;
}

HAL_StatusTypeDef USB_NegotiatedPDO(bool* result) {
  HAL_StatusTypeDef status;

  uint8_t portStatus;
  status = USB_Read_Raw(USB_PORT_STATUS_0, &portStatus, 1);
  if (status != HAL_OK) {
    return status;
  }

  bool attached = (portStatus & (1 << 0)) != 0;     // Bit 0: ATTACH_STATUS
  bool vbusPresent = (portStatus & (1 << 1)) != 0;  // Bit 1: VBUS_PRESENT
  if (!attached || !vbusPresent) {
    *result = false;
    return HAL_OK;
  }

  uint8_t peFsm;
  status = USB_Read_Raw(USB_PE_FSM, &peFsm, 1);
  if (status != HAL_OK) {
    return status;
  }

  if (peFsm != USB_PE_SNK_READY_STATE) {
    *result = false;
    return HAL_OK;
  }

  *result = true;
  return status;
}