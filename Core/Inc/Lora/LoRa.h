#include "Lora/States.h"
#include "stdint.h"
#include "string.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"

#define MAX_SIZE_TX_PACKET 240
#define UART_DELAY 50
#define RESPONSE_DELAY 500
typedef enum
{
  E22_NORMAL_MODE = 0x01,
  E22_WOR_MODE = 0x02,
  E22_CONFIG_MODE = 0x03,
  E22_SLEEP_MODE = 0x04,
  E22_MODE_INIT = 0xFF
} LoRaTypedef_MODES;


struct LoRaInitConfig
{
  GPIO_TypeDef *AUX_Port;
  GPIO_TypeDef *M0_Port;
  GPIO_TypeDef *M1_Port;
  uint16_t AUX_Pin, M0_Pin, M1_Pin;
  UART_HandleTypeDef *uartDevice;
};

void LoRaE22_Init(
    UART_HandleTypeDef *huart,
    GPIO_TypeDef *AuxPort,
    GPIO_TypeDef *M0Port,
    GPIO_TypeDef *M1Port,
    uint16_t AuxPin,
    uint16_t M0Pin,
    uint16_t M1Pin);
LoRaTypedef_STATUS LoRaE22_receiveStruct(uint8_t* structureM, uint16_t size, uint8_t RSSI, uint8_t* Vrssi);
LoRaTypedef_STATUS LoRaE22_sendStruct(uint8_t *structureM, uint16_t size);
LoRaTypedef_STATUS LoRaE22_SetMode(LoRaTypedef_MODES mode);
LoRaTypedef_STATUS LoRaE22_waitCompleteResponse(unsigned long timeout);

void _managedDelay(unsigned long timeout);

void parseSetDate(void *d, uint8_t len);
void parseTData(void *d);
void parseTError(void *d);
void parseGSCommand(void *p, uint8_t len);
void parseESP(void *p, uint8_t len);
void UART_decode_process(void *d, uint32_t len);
