#include "taskuart.h"
#include "usart.h"
#include "gpio.h"
#include "task.h"

#define STX_BUF_POS 0x0
#define ETX_BUF_POS 0x5

#define UART_ACK_CODE 0xFF
#define UART_NACK_CODE 0xFE

typedef enum _errorcode {
  ERROR_CODE_NO_ERROR         =0x0,
  ERROR_CODE_WRONG_COMMAND   = 0x1,
  ERROR_CODE_WRONG_INTERVAL  = 0x2,
  ERROR_CODE_WRONG_NUM_BLINK = 0x3,
} UART_ERROR_CODE;

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



Uart_Message packetToMessage(uint8_t *buf){
    Uart_Message msg = {0};

    return msg;
}

void uart_protocol(void) {
  static uint8_t packet[MAX_PACKET_SIZE];
  static uint8_t index = 0;
  uint8_t data;

  while (!empty_queue(&uart_rx_queue)) {
    pop_queue(&uart_rx_queue, &data);
    packet[index++] = data;

    if(index <= 6) {
      if(packet[STX_BUF_POS] == STX && packet[ETX_BUF_POS] == ETX) {

        if(calculate_checksum(&packet[], 3) != pack) return;
        Uart_Message msg = packetToMessage(packet);
     
         switch (cmd)
          {
          case CMD_LED_OFF:
            setLedOperation(LED_OFF);
            sendResponse(UART_ACK_CODE,ERROR_CODE_NO_ERROR );
            break;

          case CMD_LED_ON:
            setLedOperation(LED_OFF);
            sendResponse(UART_ACK_CODE,ERROR_CODE_NO_ERROR );
            break;

          case CMD_LED_BLINK:
            setLedOperation(LED_BLINK);
            if(d0!=){
                sendResponse(UART_NACK_CODE,ERROR_CODE_WRONG_INTERVAL );
                return;
            }
             setInterval();
            if(d1!= ){
               sendResponse(UART_NACK_CODE,ERROR_CODE_WRONG_INTERVAL );
               return;
            }
            setBlinkCount();
            sendResponse(UART_ACK_CODE,ERROR_CODE_NO_ERROR );
            break;
          default:
            sendResponse(UART_NACK_CODE,ERROR_CODE_WRONG_COMMAND );
            break;
          }
        } else {
          sendResponse(UART_ACK_CODE,ERROR_CODE_NO_ERROR );
        }
      index = 0;
      }
    }
 }

uint8_t calculate_checksum(uint8_t *buf, int len)
{
 return 0; 
}

void uart_send_packet(uint8_t cmd, uint8_t d0, uint8_t d1) {
  uint8_t packet[MAX_PACKET_SIZE];
  packet[0] = STX;
  packet[1] = cmd;
  packet[2] = d0;
  packet[3] = d1;
  packet[4] = calculate_checksum(&packet[1], 3); 
  packet[5] = ETX; 

//   for(int i = 0; i < MAX_PACKET_SIZE; i++) {
//     uart_hal_putchar(packet[i]);
//   }
    HAL_UART_Transmit(&huart2,packet,MAX_PACKET_SIZE,100);
  //HAL_UART_Transmit_IT(&huart2, &b, 1);
 
}

void sendResponse(uint8_t reply_code, UART_ERROR_CODE error_code)
{
    // uart_send_packet();
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
