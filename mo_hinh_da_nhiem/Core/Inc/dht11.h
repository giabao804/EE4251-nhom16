#ifndef DHT11_H
#define DHT11_H

#include "stm32f1xx_hal.h"

#define DHT11_PORT GPIOB
#define DHT11_PIN  GPIO_PIN_9

typedef struct {
    uint8_t temperature;
    uint8_t humidity;
} DHT11_Data;

uint8_t DHT11_Read(DHT11_Data *data);

#endif // DHT11_H
