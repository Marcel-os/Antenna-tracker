/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t DataToSend[100]; // Tablica zawierajaca dane do wyslania
uint8_t MessageCounter = 0; // Licznik wyslanych wiadomosci
uint8_t MessageLength = 0; // Zawiera dlugosc wysylanej wiadomosci
uint8_t ReceivedData[100]; // Tablica przechowujaca odebrane dane
uint8_t ReceivedDataFlag = 0; // Flaga informujaca o odebraniu danych

volatile uint16_t pulse_count_azimuth; // Licznik impulsow
volatile uint16_t positions_azimuth; // Licznik przekreconych pozycji

volatile uint16_t pulse_count_height; // Licznik impulsow
volatile uint16_t positions_height; // Licznik przekreconych pozycji

cpid_t pid_azimuth;
cpid_t pid_height;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len){
    //HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 50);
    CDC_Transmit_FS((uint8_t*)ptr, len);
    return len;
}

void parse(){
  	char header[1];
  	int32_t PWM1, PWM2, DIR1, DIR2;

  	sscanf(ReceivedData, "%s %d %d %d %d", &header, &PWM1, &PWM2, &DIR1, &DIR2);
  	if( header[0] == 'S' && PWM1 >= 0 && PWM1 < 65535 && PWM2 >= 0 && PWM2 < 65535 && (DIR1 == 1 || DIR1 == 0) && (DIR2 == 1 || DIR2 == 0) )
  	{
  		send_json(PWM1, PWM2);
  		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM1 );
  		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM2 );

  		if(DIR1 == 1){
  			HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_SET);
  			HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_RESET);
  		}

  		if(DIR1 == 0){
  			HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_SET);
  		}

  		if(DIR2 == 1){
  			HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_SET);
  			HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_RESET);
  		}

  		if(DIR2 == 0){
  			HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_SET);
  		}

  		if( PWM1 == 0 && PWM2 ==0 ){
  			HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_RESET);
  			HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_RESET);
  		}
  		send_json(PWM1, PWM2);

//	  	sprintf(DataToSend, "%d %d %d %d \r\n", PWM1, PWM2, DIR1, DIR2);
//	  	printf(DataToSend);
  	}else printf("error - zle dane \r\n");
}

void send_json(int32_t Encoder1, int32_t Encoder2){
	printf("{\"enkoder1\":%d,\"enkoder2\":%d}\r\n", Encoder1, Encoder2);
}

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  pid_init(&pid_azimuth, 150.0f, 50.0f, 0.005f, 10, 1);
  pid_azimuth.p_max = pid_scale(&pid_azimuth, 4095);
  pid_azimuth.p_min = pid_scale(&pid_azimuth, -4095);
  pid_azimuth.i_max = pid_scale(&pid_azimuth, 4095);
  pid_azimuth.i_min = pid_scale(&pid_azimuth, -4095);
  pid_azimuth.d_max = pid_scale(&pid_azimuth, 4095);
  pid_azimuth.d_min = pid_scale(&pid_azimuth, -4095);
  pid_azimuth.total_max = pid_scale(&pid_azimuth, 4095);
  pid_azimuth.total_min = pid_scale(&pid_azimuth, 0);

  pid_init(&pid_height, 150.0f, 50.0f, 0.005f, 10, 1);
  pid_height.p_max = pid_scale(&pid_height, 4095);
  pid_height.p_min = pid_scale(&pid_height, -4095);
  pid_height.i_max = pid_scale(&pid_height, 4095);
  pid_height.i_min = pid_scale(&pid_height, -4095);
  pid_height.d_max = pid_scale(&pid_height, 4095);
  pid_height.d_min = pid_scale(&pid_height, -4095);
  pid_height.total_max = pid_scale(&pid_height, 4095);
  pid_height.total_min = pid_scale(&pid_height, 0);




  //pwm_control = pid_calc(&pid, adc_value, pwm_value);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

		HAL_GPIO_WritePin(MOTOR11_GPIO_Port, MOTOR11_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR12_GPIO_Port, MOTOR12_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR21_GPIO_Port, MOTOR21_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR22_GPIO_Port, MOTOR22_Pin, GPIO_PIN_RESET);
  while (1)
  {

		pulse_count_azimuth = TIM2->CNT; // przepisanie wartosci z rejestru timera
		positions_azimuth = pulse_count_azimuth/4;

		pulse_count_height = TIM2->CNT; // przepisanie wartosci z rejestru timera
		positions_height = pulse_count_height/4;

	  if(ReceivedDataFlag == 1){
	  	ReceivedDataFlag = 0;
	  	parse();
	  }

//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 65000);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 65000);
//	  HAL_Delay(1000);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
