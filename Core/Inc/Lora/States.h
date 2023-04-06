//
// Created by ajanx on 15.05.2022.
//
#include "stdint.h"
#include "string.h"
#ifndef LORA_STATES_H
#define LORA_STATES_H

typedef enum RESPONSE_STATUS {
    E22_SUCCESS = 1,
    ERR_E22_UNKNOWN,	/* something shouldn't happen */
    ERR_E22_NOT_SUPPORT,
    ERR_E22_NOT_IMPLEMENT,
    ERR_E22_NOT_INITIAL,
    ERR_E22_INVALID_PARAM,
    ERR_E22_DATA_SIZE_NOT_MATCH,
    ERR_E22_BUF_TOO_SMALL,
    ERR_E22_TIMEOUT,
    ERR_E22_HARDWARE,
    ERR_E22_HEAD_NOT_RECOGNIZED,
    ERR_E22_NO_RESPONSE_FROM_DEVICE,
    ERR_E22_WRONG_UART_CONFIG,
    ERR_E22_WRONG_FORMAT,
    ERR_E22_PACKET_TOO_BIG
} LoRaTypedef_STATUS;

#endif //LORA_STATES_H