#ifndef TASKUART_H
#define TASKUART_H

#include <stdint.h>
#include "main.h"
#include "queue.h"

#define STX 0x02
#define ETX 0x03

#define CMD_LED_OFF 0xC0
#define CMD_LED_ON 0xC1
#define CMD_LED_BLINK 0xC2

#define MAX_PACKET_SIZE 6

extern Queue uart_rx_queue;
extern Queue uart_tx_queue;
extern uint8_t uart_hal_rx_temp;

typedef struct _message
{
  uint8_t cmd;
  uint8_t d0;
  uint8_t d1;
  uint8_t checksum;
}Uart_Message;

void uart_hal_buffer_init();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int uart_hal_getchar(uint8_t *ch);
void uart_protocol(void);
void uart_send_packet(uint8_t cmd, uint8_t d0, uint8_t d1);
void uart_hal_putchar(uint8_t data);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);




#endif