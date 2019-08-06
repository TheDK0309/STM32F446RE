#include "main.h"
#include "stm32l1xx_hal.h"
#include <string.h>
#define API_WRITE_KEY "SAYMFH2MT8GSL3W3"
#define THINGSPEAK_HOST "184.106.153.149"

void ESP8266_Init(UART_HandleTypeDef *uart1);
void Send_To_ThingSpeak(UART_HandleTypeDef *uart1, uint16_t temperature, uint16_t humidity);
