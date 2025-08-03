#include "peripheral.h"

#include "stusb_registers.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

void LEDWrite(bool LED1, float r, float g, float b) {
  TIM_HandleTypeDef* htim = LED1 ? &htim1 : &htim3;
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(r * 65535.0f));
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(g * 65535.0f));
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(b * 65535.0f));
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

// Returns 0, 1, 2 for PDO 1, 2, 3 respectively
HAL_StatusTypeDef USB_NegotiatedPower(bool* negotiated) {
  // Read RDO_REG_STATUS_3 to get the "Object position" (Source's PDO index)
  uint8_t rdo_status_3_raw;
  HAL_StatusTypeDef status =
      USB_Read_Raw(USB_RDO_REG_STATUS_3, &rdo_status_3_raw, 1);
  if (status != HAL_OK) {
    return status;
  }
  uint8_t source_pdo_index = (rdo_status_3_raw >> 4) & 0x07;
  if (source_pdo_index <= 1) {
    *negotiated = false;
    return HAL_OK;
  }

  *negotiated = true;
  return HAL_OK;
}

HAL_StatusTypeDef USB_ReadPDO_Raw(uint8_t pdo_num, uint32_t* result) {
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

  *result = pdoData;
  return HAL_OK;
}

// 0 for PDO 1, 1 for PDO 2, 2 for PDO 3
HAL_StatusTypeDef USB_ReadPDO(uint8_t pdo_num, float* voltage, float* current) {
  uint32_t pdoData;
  HAL_StatusTypeDef status = USB_ReadPDO_Raw(pdo_num, &pdoData);
  if (status != HAL_OK) {
    return status;
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

  /*bool attached = (portStatus & (1 << 0)) != 0;     // Bit 0: ATTACH_STATUS
  bool vbusPresent = (portStatus & (1 << 1)) != 0;  // Bit 1: VBUS_PRESENT
  printf("USB Port Status - Attached: %d, VBUS Present: %d\n", attached,
         vbusPresent);
  if (!attached || !vbusPresent) {
    *result = false;
    return HAL_OK;
  }*/

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

// Can only do PDO num 1 & 2 (PDOs 2 and 3)
HAL_StatusTypeDef USB_WritePDO(uint8_t pdo_num, float voltage, float current) {
  uint32_t pdoData;
  HAL_StatusTypeDef status = USB_ReadPDO_Raw(pdo_num, &pdoData);
  if (status != HAL_OK) {
    return status;
  }

  uint32_t volt = (uint32_t)(voltage * 20.0f);
  uint32_t curr = (uint32_t)(current / 0.01f) & 0x3FF;

  // Clear the voltage and current bits
  pdoData &= ~(0x3FF << 10);  // Clear voltage bits
  pdoData &= ~0x3FF;          // Clear current bits

  // Set the new voltage and current bits
  pdoData |= (volt << 10);  // Set voltage bits
  pdoData |= curr;          // Set current bits

  // Write PDO
  USB_Write_Raw(0x85 + (pdo_num * 4), (uint8_t*)&pdoData, 4);
  return status;
}

// Write current PDO settings to NVM (all 3 PDOs)
HAL_StatusTypeDef USB_WriteNVMFromPDOs() {
  uint8_t sector[5][8] = {0};  // Local buffer for NVM data
  float voltage[3] = {0};
  float current[3] = {0};
  uint8_t nvmCurrent[3] = {0};
  uint32_t digitalVoltage = 0;
  uint8_t buf[1];
  HAL_StatusTypeDef status;

  // --- STEP 1: READ ALL CURRENT NVM SECTORS INTO 'sector' ARRAY ---
  // Enter NVM Read Mode
  buf[0] = USB_FTP_CUST_PASSWORD;
  status = USB_Write_Raw(USB_FTP_CUST_PASSWORD_REG, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = 0;  // NVM internal controller reset
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N;  // Set PWR and RST_N bits
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;
  // --- End of CUST_EnterReadMode equivalent for read ---

  for (uint8_t i = 0; i < 5; i++) {
    // Re-assert PWR and RST_N for each sector read operation
    buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N;
    status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
    if (status != HAL_OK) return status;

    buf[0] = (USB_READ & USB_FTP_CUST_OPCODE);  // Set Read Sectors Opcode
    status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
    if (status != HAL_OK) return status;

    buf[0] = (i & USB_FTP_CUST_SECT) | USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N |
             USB_FTP_CUST_REQ;
    status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);  // Load Read Sectors Opcode
    if (status != HAL_OK) return status;

    do {
      status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);  // Wait for execution
      if (status != HAL_OK) return status;
    } while (buf[0] & USB_FTP_CUST_REQ);

    status = USB_Read_Raw(USB_RW_BUFFER, &sector[i][0],
                          8);  // Read 8 bytes into sector
    if (status != HAL_OK) return status;
  }

  // --- Exit Read Mode (inline CUST_ExitTestMode equivalent) ---
  buf[0] = USB_FTP_CUST_RST_N;
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;
  buf[0] = 0x00;
  status = USB_Write_Raw(USB_FTP_CUST_PASSWORD_REG, buf, 1);
  if (status != HAL_OK) return status;
  // --- End of CUST_ExitTestMode equivalent for read ---

  // --- STEP 2: MODIFY PDO BITS WITHIN THE 'sector' ARRAY ---

  // Read current PDOs from RAM and convert to NVM format
  for (uint8_t i = 0; i < 3; i++) {
    if (USB_ReadPDO(i, &voltage[i], &current[i]) != HAL_OK) return HAL_ERROR;

    // Constrain current values
    if (current[i] > 5.0f) current[i] = 5.0f;
    if (current[i] < 0.5f)
      nvmCurrent[i] = 0;
    else if (current[i] <= 3.0f)
      nvmCurrent[i] = (uint8_t)(4 * current[i] - 1);
    else
      nvmCurrent[i] = (uint8_t)(2 * current[i] + 5);

    // Constrain voltage values (though PDO1 is fixed at 5V)
    if (voltage[i] < 5.0f)
      voltage[i] = 5.0f;
    else if (voltage[i] > 20.0f)
      voltage[i] = 20.0f;
  }

  // Pack current and voltage into sector buffer (preserving other bits)
  // PDO1 current (bits 4:7 of sector 3, byte 2)
  sector[3][2] &= 0x0F;  // Clear bits 4:7
  sector[3][2] |= (nvmCurrent[0] << 4);

  // PDO2 current (bits 0:3 of sector 3, byte 4)
  sector[3][4] &= 0xF0;  // Clear bits 0:3
  sector[3][4] |= nvmCurrent[1];

  // PDO3 current (bits 4:7 of sector 3, byte 5)
  sector[3][5] &= 0x0F;  // Clear bits 4:7
  sector[3][5] |= (nvmCurrent[2] << 4);

  // PDO2 voltage (10-bit value across sector 4, byte 0 bits 6:7 and byte 1 bits
  // 0:7)
  digitalVoltage = (uint32_t)(voltage[1] * 20.0f);
  sector[4][0] &= 0x3F;  // Clear bits 6:7 of byte 0
  sector[4][0] |=
      ((digitalVoltage & 0x03) << 6);    // Load bits 0:1 of digitalVoltage
  sector[4][1] = (digitalVoltage >> 2);  // Load bits 2:9 of digitalVoltage

  // PDO3 voltage (10-bit value across sector 4, byte 2 bits 0:7 and byte 3 bits
  // 0:1)
  digitalVoltage = (uint32_t)(voltage[2] * 20.0f);
  sector[4][2] = digitalVoltage & 0xFF;   // Load bits 0:7 of digitalVoltage
  sector[4][3] &= 0xFC;                   // Clear bits 0:1 of byte 3
  sector[4][3] |= (digitalVoltage >> 8);  // Load bits 8:9 of digitalVoltage

  // Load highest priority PDO number from RAM (sector 3, byte 2, bits 2:3)
  // This ensures the active PDO priority is also saved to NVM.
  uint8_t pdo_num_val[1];
  status = USB_Read_Raw(USB_DPM_PDO_NUMB, pdo_num_val, 1);
  if (status != HAL_OK) return status;
  sector[3][2] &= 0xF9;  // Clear bits 2:3
  sector[3][2] |=
      (pdo_num_val[0] << 1) & 0x06;  // Set bits 2:3 from DPM_PDO_NUMB

  // --- STEP 3: PERFORM THE NVM WRITE SEQUENCE ---

  // Enter NVM write mode (inline CUST_EnterWriteMode equivalent)
  buf[0] = USB_FTP_CUST_PASSWORD;
  status = USB_Write_Raw(USB_FTP_CUST_PASSWORD_REG, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = 0;  // This register must be NULL for Partial Erase feature
  status = USB_Write_Raw(USB_RW_BUFFER, buf, 1);
  if (status != HAL_OK) return status;

  // NVM Power-up Sequence
  buf[0] = 0;  // NVM internal controller reset
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N;  // Set PWR and RST_N bits
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = ((USB_SECTOR_0 | USB_SECTOR_1 | USB_SECTOR_2 | USB_SECTOR_3 |
             USB_SECTOR_4)
                << 3 &
            USB_FTP_CUST_SER) |
           (USB_WRITE_SER & USB_FTP_CUST_OPCODE);
  status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N | USB_FTP_CUST_REQ;
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  do {
    status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);  // Wait for execution
    if (status != HAL_OK) return status;
  } while (buf[0] & USB_FTP_CUST_REQ);

  buf[0] = USB_SOFT_PROG_SECTOR & USB_FTP_CUST_OPCODE;
  status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N | USB_FTP_CUST_REQ;
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  do {
    status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);  // Wait for execution
    if (status != HAL_OK) return status;
  } while (buf[0] & USB_FTP_CUST_REQ);

  buf[0] = USB_ERASE_SECTOR & USB_FTP_CUST_OPCODE;
  status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N | USB_FTP_CUST_REQ;
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  do {
    status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);  // Wait for execution
    if (status != HAL_OK) return status;
  } while (buf[0] & USB_FTP_CUST_REQ);
  // --- End of CUST_EnterWriteMode equivalent for write ---

  // Write all sectors (inline CUST_WriteSector equivalent for each sector)
  for (uint8_t s = 0; s < 5; s++) {
    status = USB_Write_Raw(USB_RW_BUFFER, sector[s], 8);
    if (status != HAL_OK) return status;

    buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N;
    status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
    if (status != HAL_OK) return status;

    buf[0] = USB_WRITE_PL & USB_FTP_CUST_OPCODE;
    status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
    if (status != HAL_OK) return status;

    buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N | USB_FTP_CUST_REQ;
    status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
    if (status != HAL_OK) return status;

    do {
      status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);
      if (status != HAL_OK) return status;
    } while (buf[0] & USB_FTP_CUST_REQ);

    buf[0] = USB_PROG_SECTOR & USB_FTP_CUST_OPCODE;
    status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
    if (status != HAL_OK) return status;

    buf[0] = (s & USB_FTP_CUST_SECT) | USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N |
             USB_FTP_CUST_REQ;
    status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
    if (status != HAL_OK) return status;

    do {
      status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);
      if (status != HAL_OK) return status;
    } while (buf[0] & USB_FTP_CUST_REQ);
  }

  // --- STEP 4: EXIT TEST MODE AND SOFT RESET ---

  // Exit test mode (inline CUST_ExitTestMode equivalent)
  buf[0] = USB_FTP_CUST_RST_N;
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;
  buf[0] = 0x00;
  status = USB_Write_Raw(USB_FTP_CUST_PASSWORD_REG, buf, 1);
  if (status != HAL_OK) return status;

  // Soft reset to load new NVM settings
  uint8_t resetBuf[1];
  resetBuf[0] = 0x0D;  // SOFT_RESET
  status = USB_Write_Raw(USB_TX_HEADER_LOW, resetBuf, 1);
  if (status != HAL_OK) return status;
  resetBuf[0] = 0x26;  // SEND_COMMAND
  status = USB_Write_Raw(USB_PD_COMMAND_CTRL, resetBuf, 1);
  if (status != HAL_OK) return status;

  return HAL_OK;
}
HAL_StatusTypeDef USB_WriteDefaultNVM() {
  uint8_t default_sector[5][8] = {
      {0x00, 0x00, 0xB0, 0xAA, 0x00, 0x45, 0x00, 0x00},
      {0x10, 0x40, 0x9C, 0x1C, 0xFF, 0x01, 0x3C, 0xDF},
      {0x02, 0x40, 0x0F, 0x00, 0x32, 0x00, 0xFC, 0xF1},
      {0x00, 0x19, 0x56, 0xAF, 0xF5, 0x35, 0x5F, 0x00},
      {0x00, 0x4B, 0x90, 0x21, 0x43, 0x00, 0x40, 0xFB}};
  uint8_t buf[1];
  HAL_StatusTypeDef status;

  // --- Enter NVM write mode (inline CUST_EnterWriteMode equivalent) ---
  buf[0] = USB_FTP_CUST_PASSWORD;
  status = USB_Write_Raw(USB_FTP_CUST_PASSWORD_REG, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = 0;  // This register must be NULL for Partial Erase feature
  status = USB_Write_Raw(USB_RW_BUFFER, buf, 1);
  if (status != HAL_OK) return status;

  // NVM Power-up Sequence
  buf[0] = 0;  // NVM internal controller reset
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N;  // Set PWR and RST_N bits
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = ((USB_SECTOR_0 | USB_SECTOR_1 | USB_SECTOR_2 | USB_SECTOR_3 |
             USB_SECTOR_4)
                << 3 &
            USB_FTP_CUST_SER) |
           (USB_WRITE_SER & USB_FTP_CUST_OPCODE);
  status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N | USB_FTP_CUST_REQ;
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  do {
    status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);  // Wait for execution
    if (status != HAL_OK) return status;
  } while (buf[0] & USB_FTP_CUST_REQ);

  buf[0] = USB_SOFT_PROG_SECTOR & USB_FTP_CUST_OPCODE;
  status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N | USB_FTP_CUST_REQ;
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  do {
    status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);  // Wait for execution
    if (status != HAL_OK) return status;
  } while (buf[0] & USB_FTP_CUST_REQ);

  buf[0] = USB_ERASE_SECTOR & USB_FTP_CUST_OPCODE;
  status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
  if (status != HAL_OK) return status;

  buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N | USB_FTP_CUST_REQ;
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;

  do {
    status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);  // Wait for execution
    if (status != HAL_OK) return status;
  } while (buf[0] & USB_FTP_CUST_REQ);
  // --- End of CUST_EnterWriteMode equivalent for write ---

  // Write all sectors (inline CUST_WriteSector equivalent for each sector)
  for (uint8_t s = 0; s < 5; s++) {
    status = USB_Write_Raw(USB_RW_BUFFER, default_sector[s], 8);
    if (status != HAL_OK) return status;

    buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N;
    status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
    if (status != HAL_OK) return status;

    buf[0] = USB_WRITE_PL & USB_FTP_CUST_OPCODE;
    status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
    if (status != HAL_OK) return status;

    buf[0] = USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N | USB_FTP_CUST_REQ;
    status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
    if (status != HAL_OK) return status;

    do {
      status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);
      if (status != HAL_OK) return status;
    } while (buf[0] & USB_FTP_CUST_REQ);

    buf[0] = USB_PROG_SECTOR & USB_FTP_CUST_OPCODE;
    status = USB_Write_Raw(USB_FTP_CTRL_1, buf, 1);
    if (status != HAL_OK) return status;

    buf[0] = (s & USB_FTP_CUST_SECT) | USB_FTP_CUST_PWR | USB_FTP_CUST_RST_N |
             USB_FTP_CUST_REQ;
    status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
    if (status != HAL_OK) return status;

    do {
      status = USB_Read_Raw(USB_FTP_CTRL_0, buf, 1);
      if (status != HAL_OK) return status;
    } while (buf[0] & USB_FTP_CUST_REQ);
  }

  // --- Exit test mode (inline CUST_ExitTestMode equivalent) ---
  buf[0] = USB_FTP_CUST_RST_N;
  status = USB_Write_Raw(USB_FTP_CTRL_0, buf, 1);
  if (status != HAL_OK) return status;
  buf[0] = 0x00;
  status = USB_Write_Raw(USB_FTP_CUST_PASSWORD_REG, buf, 1);
  if (status != HAL_OK) return status;

  // Soft reset to load new NVM settings
  uint8_t resetBuf[1];
  resetBuf[0] = 0x0D;  // SOFT_RESET
  status = USB_Write_Raw(USB_TX_HEADER_LOW, resetBuf, 1);
  if (status != HAL_OK) return status;
  resetBuf[0] = 0x26;  // SEND_COMMAND
  status = USB_Write_Raw(USB_PD_COMMAND_CTRL, resetBuf, 1);
  if (status != HAL_OK) return status;

  return HAL_OK;
}

// DAC
extern DAC_HandleTypeDef hdac;
void EnableReg(bool chan1, bool en) {
  if (chan1) {
    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin,
                      en ? GPIO_PIN_SET : GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin,
                      en ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}
const float ANALOG_SCALE = (3.3f / 4095.0f) * ((20000.0f + 3300.0f) / 3300.0f);