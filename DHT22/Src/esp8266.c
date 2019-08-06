#include "esp8266.h"

void ESP8266_Init(UART_HandleTypeDef *uart1)
{
	uint8_t command0[] = "AT\r\n";
	uint8_t command1[] = "AT+RESTORE\r\n";
	uint8_t command2[] = "AT+CWMODE=3\r\n";
	uint8_t command3[] = "AT+CWJAP=\"Mi Phone\",\"11223456\"\r\n";
	uint8_t allow_multi[] = "AT+CIPMUX=0\r\n";
	HAL_UART_Transmit(uart1, command0, sizeof(command0), 100);
	HAL_Delay(1000); // 1 sec
	HAL_UART_Transmit(uart1, command1, sizeof(command1), 100);
	HAL_Delay(2000); // 2 sec
	HAL_UART_Transmit(uart1, command2, sizeof(command2), 100);
	HAL_Delay(1000); // 1 sec
	HAL_UART_Transmit(uart1, command3, sizeof(command3), 100);
	HAL_Delay(10000); // 10 sec
	HAL_UART_Transmit(uart1, allow_multi, strlen((char *)allow_multi), 100);
	HAL_Delay(1000);
}
void Send_To_ThingSpeak(UART_HandleTypeDef *uart1, uint16_t temperature, uint16_t humidity)
{
	uint8_t cip_start[] = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n";
	uint8_t cip_send[] = "AT+CIPSEND=51\r\n"; // 51 bytes
	uint8_t cip_close[] = "AT+CIPCLOSE\r\n";
	uint8_t data1[60];
	uint8_t data2[60];

	sprintf((char *)data1, "GET /update?api_key=%s&field%d=%d\r\n", API_WRITE_KEY, 1, temperature / 10);
	sprintf((char *)data2, "GET /update?api_key=%s&field%d=%d\r\n", API_WRITE_KEY, 2, humidity / 10);

	HAL_UART_Transmit(uart1, cip_close, strlen((char *)cip_close), 100);
	HAL_Delay(2000);
	HAL_UART_Transmit(uart1, cip_start, strlen((char *)cip_start), 100);
	HAL_Delay(5000);
	HAL_UART_Transmit(uart1, cip_send, strlen((char *)cip_send), 100);
	HAL_Delay(1000);
	HAL_UART_Transmit(uart1, data1, strlen((char *)data1), 100);
	HAL_Delay(20000);

	HAL_UART_Transmit(uart1, cip_close, strlen((char *)cip_close), 100);
	HAL_Delay(2000);
	HAL_UART_Transmit(uart1, cip_start, strlen((char *)cip_start), 100);
	HAL_Delay(5000);
	HAL_UART_Transmit(uart1, cip_send, strlen((char *)cip_send), 100);
	HAL_Delay(1000);
	HAL_UART_Transmit(uart1, data2, strlen((char *)data2), 100);
	HAL_Delay(20000);

}
