#include "taskuart.h"
#include "usart.h"
#include "gpio.h"
#include "task.h"

Queue uart_rx_queue; // PC -> STM
Queue uart_tx_queue; // STM -> PC
uint8_t uart_hal_rx_temp; 

void uart_hal_buffer_init() {
  init_queue(&uart_rx_queue);
  init_queue(&uart_tx_queue);

  HAL_UART_Receive_IT(&huart2, &uart_hal_rx_temp, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if(huart->Instance == USART2) {

    push_queue(&uart_rx_queue, uart_hal_rx_temp);
    HAL_UART_Receive_IT(&huart2, &uart_hal_rx_temp, 1);
  }
}


int uart_hal_getchar(uint8_t *ch) { // PC > STM 
  uint8_t data;
  if(pop_queue(&uart_rx_queue, &data)) {
    *ch = data;
    return 1;
  }
  return 0;
}

void uart_protocol(void) {
  static uint8_t packet[MAX_PACKET_SIZE];
  static uint8_t index = 0;
  uint8_t data;

  while (!empty_queue(&uart_rx_queue)) {
    pop_queue(&uart_rx_queue, &data);
    packet[index++] = data;

    if(index >= 6) {
      if(packet[0] == STX && packet[5] == ETX) {
        uint8_t cmd = packet[1];
        uint8_t d0 = packet[2];
        uint8_t d1 = packet[3];
        uint8_t sum = packet[4];

        if(sum == 0xff - (cmd + d0 + d1)) {
         switch (cmd)
          {
          case CMD_LED_OFF:
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            uart_send_packet(0xff, 0, 0);
            break;

          case CMD_LED_ON:
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            uart_send_packet(0xff, 0, 0);
            break;

          case CMD_LED_BLINK:
            if(d0 > 0 && d0 <=50) { 
              interval = (int)d0 * 100;
              if(d1 == 0) {
                //continuous
                blink_count = 0xff;
              } else if(d1 == 0xff) {
                //stop
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                blink_count = 0;
              } else if(d1 <= 200) {
                //toggle 
                blink_count = (uint8_t)d1;

              } else {
                uart_send_packet(0xfe, 0x03, 0); // Wrong Number of Blink
              }
            } else {
              uart_send_packet(0xfe, 0x02, 0); // Wrong Interval
            }
            uart_send_packet(0xff, 0, 0);
            break;
        
          default:
            uart_send_packet(0xfe, 0x01, 0); // Wrong Command
            break;
          }
        } else {
          uart_send_packet(0xfe, 0x01, 0);
        }
      index = 0;
      }
    }
  }
 }

 void uart_send_packet(uint8_t cmd, uint8_t d0, uint8_t d1) {
  uint8_t packet[MAX_PACKET_SIZE];
  packet[0] = STX;
  packet[1] = cmd;
  packet[2] = d0;
  packet[3] = d1;
  packet[4] = 0xff - (cmd + d0 + d1);
  packet[5] = ETX; 

//   for(int i = 0; i < MAX_PACKET_SIZE; i++) {
//     uart_hal_putchar(packet[i]);
//   }
    HAL_UART_Transmit(&huart2,packet,MAX_PACKET_SIZE,100);
  //HAL_UART_Transmit_IT(&huart2, &b, 1);
 
}

// void uart_hal_putchar(uint8_t data) { // STM > PC
//   push_queue(&uart_tx_queue, data);
//   if(uart_tx_queue.size == 1) {
//     uint8_t byte;
//     if(pop_queue(&uart_tx_queue, &byte)) {
//       HAL_UART_Transmit_IT(&huart2, &byte, 1);
//     }
//   }
// }

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//   if(huart->Instance == USART2) {
//     uint8_t byte;
//     if(pop_queue(&uart_tx_queue, &byte)) {
//       HAL_UART_Transmit_IT(&huart2, &byte, 1);
//     }
//   }
// }
