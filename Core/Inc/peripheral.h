#ifndef __PERIPHERAL_H
#define __PERIPHERAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
void LEDWrite(bool LED1, float r, float g, float b);
HAL_StatusTypeDef USB_ReadPDO(uint8_t pdo_num, float* voltage, float* current);
HAL_StatusTypeDef USB_NegotiatedPower(
    bool* negotiated);  // Gives whether we are not on PDO 1
HAL_StatusTypeDef USB_NegotiatedPDO(bool* result);
HAL_StatusTypeDef USB_WriteNVMFromPDOs();
HAL_StatusTypeDef USB_WritePDO(uint8_t pdo_num, float voltage, float current);
HAL_StatusTypeDef USB_WriteDefaultNVM();
#ifdef __cplusplus
}
#endif

#endif /* __PERIPHERAL_H */