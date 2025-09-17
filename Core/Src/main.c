/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define STX 0x02
#define ETX 0x03

#define CMD_LED_OFF 0xC0
#define CMD_LED_ON 0xC1
#define CMD_LED_BLINK 0xC2

#define UART_TX_BUFFER_SIZE 128
#define UART_RX_BUFFER_SIZE 128

typedef struct {
  uint8_t buffer[UART_RX_BUFFER_SIZE];
  uint8_t temp;
  volatile uint16_t input_p;
  volatile uint16_t output_p;
} UART_RingBufferRx;

typedef struct {
  uint8_t buffer[UART_TX_BUFFER_SIZE];
  volatile uint16_t input_p;
  volatile uint16_t output_p;
  uint8_t busy;
} UART_RingBufferTx;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_data;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);

  uart_hal_buffer_init();
  //print_cmd();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    uart_protocol();
    HAL_Delay(100);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == B1_Pin) {
    push_button();
	}
}*/

UART_RingBufferTx uart_hal_tx;
UART_RingBufferRx uart_hal_rx;

void uart_hal_buffer_init() {
  uart_hal_rx.input_p = uart_hal_tx.input_p = 0;
  uart_hal_rx.output_p = uart_hal_tx.output_p = 0;
  uart_hal_tx.busy = 0;

  HAL_UART_Receive_IT(&huart2, &uart_hal_rx.temp, 1);
}

int uart_hal_getchar(uint8_t *data) { // PC > STM 
  if (uart_hal_rx.input_p == uart_hal_rx.output_p) return 0; 
  *data = uart_hal_rx.buffer[uart_hal_rx.output_p++];
  if(uart_hal_rx.output_p >= UART_RX_BUFFER_SIZE) {
    uart_hal_rx.output_p = 0; 
  }
  return 1;
}

void uart_hal_putchar(uint8_t data) { // STM > PC
  uart_hal_tx.buffer[uart_hal_tx.input_p++] = data;
  if(uart_hal_tx.input_p >= UART_TX_BUFFER_SIZE) { 
    uart_hal_tx.input_p = 0;
  } 

  if (!uart_hal_tx.busy) {
        uart_hal_tx.busy = 1;
        uint8_t data = uart_hal_tx.buffer[uart_hal_tx.output_p++];
        if (uart_hal_tx.output_p >= UART_TX_BUFFER_SIZE) uart_hal_tx.output_p = 0;
        HAL_UART_Transmit_IT(&huart2, &data, 1);
    }
}

void uart_hal_send(uint8_t *data, uint16_t len) {
  for(int i = 0; i < len; i++) {
    uart_hal_putchar(data[i]);
  }
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
  if(htim->Instance==TIM4) {
    task_timer();
  }
}

/*int __io_putchar (int ch) {
    (void) HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 100);
    return ch;
  }

 void print_cmd(void) {
  printf("----------\r\n");
  printf("1: LED On\r\n");
  printf("2: LED Off\r\n");
  printf("3: LED Blink\r\n");
  printf("----------\r\nEnter Command:\r\n");
}*/ 

  void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if(huart->Instance == USART2) {
    if(uart_hal_tx.input_p != uart_hal_tx.output_p) {
      uint8_t data = uart_hal_tx.buffer[uart_hal_tx.output_p++];
      if(uart_hal_tx.output_p >= UART_TX_BUFFER_SIZE) {
        uart_hal_tx.output_p = 0;
      }
      HAL_UART_Transmit_IT(&huart2, &data, 1);
    } else {
      uart_hal_tx.busy = 0;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if(huart->Instance == USART2) {
    uart_hal_rx.buffer[uart_hal_rx.input_p++] = uart_hal_rx.temp; 
    if(uart_hal_rx.input_p >= UART_RX_BUFFER_SIZE) {
      uart_hal_rx.input_p = 0;
    }
    HAL_UART_Receive_IT(huart, &uart_hal_rx.temp, 1); 
  }
}

void uart_send_packet(uint8_t cmd, uint8_t d0, uint8_t d1) {
  uint8_t packet[6];
  packet[0] = STX;
  packet[1] = cmd;
  packet[2] = d0;
  packet[3] = d1;
  packet[4] = 0xff - (cmd + d0 + d1);
  packet[5] = ETX; 

  uart_hal_send(packet, 6);
}

void uart_protocol(void) {
  static uint8_t packet[6];
  static uint8_t index = 0;
  uint8_t data;

  while (uart_hal_getchar(&data)) {
    if(index == 0 && data != STX) {
      continue;
    } 
    packet[index++] = data;

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

        /*case CMD_LED_BLINK: //??????
          if(d1 == 0) {
            task_led();
          } else {
            for(int i = 0; i < d1; i++) {
              task_led();
            }
          }
          uart_send_packet(0xff, 0, 0);*/
        
        default:
          uart_send_packet(0xfe, 0x01, 0);
          break;
        }
      } else {
        uart_send_packet(0xfe, 0x01, 0);
      }
    }
    index = 0;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
