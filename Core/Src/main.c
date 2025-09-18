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
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STX 0x02
#define ETX 0x03

#define CMD_LED_OFF 0xC0
#define CMD_LED_ON 0xC1
#define CMD_LED_BLINK 0xC2

#define MAX_PACKET_SIZE 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Queue uart_rx_queue; // PC -> STM
Queue uart_tx_queue; // STM -> PC
uint8_t uart_hal_rx_temp; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

void uart_hal_buffer_init() {
  init_queue(&uart_rx_queue);
  init_queue(&uart_tx_queue);

  HAL_UART_Receive_IT(&huart2, &uart_hal_rx_temp, 1);
}

int uart_hal_getchar(uint8_t *ch) { // PC > STM 
  int data;
  if(pop_queue(&uart_rx_queue, &data)) {
    *ch = (uint8_t)data;
    return 1;
  }
  return 0;
}

void uart_hal_putchar(uint8_t data) { // STM > PC
  push_queue(&uart_tx_queue, data);
  if(uart_tx_queue.size = 1) {
    int byte;
    if(pop_queue(&uart_tx_queue, &byte)) {
      uint8_t b = (uint8_t)byte;
      HAL_UART_Transmit_IT(&huart2, &b, 1);
    }
  }
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
  if(htim->Instance==TIM4) {
    task_timer();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if(huart->Instance == USART2) {
    int byte;
    if(pop_queue(&uart_tx_queue, &byte)) {
      uint8_t b = (uint8_t)byte;
      HAL_UART_Transmit_IT(&huart2, &b, 1);
    }
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if(huart->Instance == USART2) {
    push_queue(&uart_rx_queue, uart_hal_rx_temp);
    HAL_UART_Receive_IT(&huart2, &uart_hal_rx_temp, 1);
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

  for(int i = 0; i < MAX_PACKET_SIZE; i++) {
    uart_hal_putchar(packet[i]);
  }
}

void uart_protocol(void) {
  static uint8_t packet[MAX_PACKET_SIZE];
  static uint8_t index = 0;
  int data;

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
      index = 0;
      }
    }
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
