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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "lsm6dsox.h"
#include "custom_bus.h"
#include "custom_conf.h"
#include "custom_errno.h"

#include "gesture_model.h"
#include "gesture_model_data.h"

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
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
LSM6DSOX_Object_t MotionSensor;

// memory used to hold intermediate values for neural network
AI_ALIGNED(4) ai_u8 activations[AI_GESTURE_MODEL_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(4) ai_u8 in_data[AI_GESTURE_MODEL_IN_1_SIZE_BYTES];
AI_ALIGNED(4) ai_u8 out_data[AI_GESTURE_MODEL_OUT_1_SIZE_BYTES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

static void MEMS_Init(void);

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
  LSM6DSOX_AxesRaw_t acc_axes;
  LSM6DSOX_AxesRaw_t gyro_axes;

  uint8_t dataRdy;

  const float accThreshold = 2500.0; // in MilliG
  const uint8_t numSamples = 104;    // 1 second of data

  float accSum;

  uint8_t samplesRead = numSamples;
  uint16_t i = 0;

  // Vektor für Ausgangswerte
  float y_val[2];

  // Zeiger auf Modell
  ai_handle network = AI_HANDLE_NULL;

  // Zeiger auf Ein-/Ausgangsdatenpuffer
  ai_buffer *ai_input;
  ai_buffer *ai_output;

  // Hilfsvariablen
  ai_error err;
  ai_network_report report;
  ai_i32 nbatch;

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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(3);

  HAL_TIM_Base_Start(&htim16);
  uint32_t timestamp;
  uint32_t duration;

  printf("\r\nTinyML IMU Demo: Gesture detection\r\n");

  // Init LSM6DSOX IMU
  MEMS_Init();

  // Zeiger auf Aktivierungsfunktionen
  const ai_handle acts[] = {activations};

  // Instanz des neuronalen Netzes erzeugen
  err = ai_gesture_model_create_and_init(&network, acts, NULL);

  // Prüfen auf Fehler
  if (err.type != AI_ERROR_NONE) {
    printf("Error: ai_gesture_model_create_and_init()\r\n");
  } else {
    printf("Success: model created and initialized\r\n");
  }

  if (ai_gesture_model_get_report(network, &report) != true) {
    printf("Error: failed to get report\r\n");
  }

  ai_input  = &report.inputs[0];
  ai_output = &report.outputs[0];

  // Eingänge und Ausgänge mit Modell verbinden
  ai_input[0].data  = AI_HANDLE_PTR(in_data);
  ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	while (samplesRead == numSamples) {

	  // check, if new data is available
	  LSM6DSOX_ACC_Get_DRDY_Status(&MotionSensor, &dataRdy);

	  if (dataRdy) {

		// retrieve data
	    LSM6DSOX_ACC_GetAxesRaw(&MotionSensor, &acc_axes);

	    accSum = fabs(acc_axes.x * LSM6DSOX_ACC_SENSITIVITY_FS_4G) +
	    		 fabs(acc_axes.y * LSM6DSOX_ACC_SENSITIVITY_FS_4G) +
				 fabs(acc_axes.z * LSM6DSOX_ACC_SENSITIVITY_FS_4G);

	    //printf("%.1f\r\n", accSum);

	    // check, if accSum is above threshold
	    if (accSum >= accThreshold) {
	      samplesRead = 0;
	      i = 0;
	    }

	    // clear dataRdy
	    dataRdy = 0;
	  }
	}

	while (samplesRead < numSamples) {

	  // check, if new data is available
	  LSM6DSOX_ACC_Get_DRDY_Status(&MotionSensor, &dataRdy);

      if (dataRdy) {

    	// clear dataRdy
    	dataRdy = 0;

    	// retrieve new data
	    LSM6DSOX_ACC_GetAxesRaw(&MotionSensor, &acc_axes);
		LSM6DSOX_GYRO_GetAxesRaw(&MotionSensor, &gyro_axes);

		samplesRead++;

		((ai_float*)in_data)[i]   = (ai_float)((acc_axes.x * LSM6DSOX_ACC_SENSITIVITY_FS_4G) + 4000.0)/8000.0;
		((ai_float*)in_data)[i+1] = (ai_float)((acc_axes.y * LSM6DSOX_ACC_SENSITIVITY_FS_4G) + 4000.0)/8000.0;
		((ai_float*)in_data)[i+2] = (ai_float)((acc_axes.z * LSM6DSOX_ACC_SENSITIVITY_FS_4G) + 4000.0)/8000.0;
		((ai_float*)in_data)[i+3] = (ai_float)(gyro_axes.x + 8000)/16000;
		((ai_float*)in_data)[i+4] = (ai_float)(gyro_axes.y + 8000)/16000;
		((ai_float*)in_data)[i+5] = (ai_float)(gyro_axes.z + 8000)/16000;

		i = i + 6;

		// if last sample has been read, start inference
		if (samplesRead == numSamples) {

          timestamp = htim16.Instance->CNT;
          nbatch = ai_gesture_model_run(network, ai_input, ai_output);
          duration = htim16.Instance->CNT - timestamp;

          if (nbatch != 1) {
        	printf("Error: could not run inference\r\n");
          } else {
        	y_val[0] = ((float*)out_data)[0];
        	y_val[1] = ((float*)out_data)[1];

        	if (y_val[0] > y_val[1]) {
        	  printf("%.3f | %.3f | Duration: %.3f ms | Gesture: Punch\r\n", y_val[0], y_val[1], (duration/1000.0));
        	} else {
        	  printf("%.3f | %.3f | Duration: %.3f ms | Gesture: Flex\r\n", y_val[0], y_val[1], (duration/1000.0));
        	}
          }
		}
	  }
	}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 80-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void MEMS_Init(void) {

  LSM6DSOX_IO_t io_ctx;
  uint8_t id;

  io_ctx.BusType  = LSM6DSOX_I2C_BUS;
  io_ctx.Address  = LSM6DSOX_I2C_ADD_L;
  io_ctx.Init     = BSP_I2C1_Init;
  io_ctx.DeInit   = BSP_I2C1_DeInit;
  io_ctx.ReadReg  = BSP_I2C1_ReadReg;
  io_ctx.WriteReg = BSP_I2C1_WriteReg;
  io_ctx.GetTick  = BSP_GetTick;

  LSM6DSOX_RegisterBusIO(&MotionSensor, &io_ctx);

  // read the LSM6DSOX WHO_AM_I register
  LSM6DSOX_ReadID(&MotionSensor, &id);
  if (id != LSM6DSOX_ID) {
	Error_Handler();
  } else {
	printf("Found IMU: LSM6DSOX\r\n");
  }

  // initialize LSM6DSOX sensor
  LSM6DSOX_Init(&MotionSensor);

  // configure the LSM6DSOX accelerometer
  LSM6DSOX_ACC_SetOutputDataRate(&MotionSensor, 104.0f);  // Sampling rate: 104 Hz
  LSM6DSOX_ACC_SetFullScale(&MotionSensor, 4);            // [-4000mG; +4000mG]
  LSM6DSOX_ACC_Enable(&MotionSensor);                     // enable accelerometer

  // configure the LSM6DSOX gyroscope
  LSM6DSOX_GYRO_SetOutputDataRate(&MotionSensor, 104.0f); // Sampling rate: 104 Hz
  LSM6DSOX_GYRO_SetFullScale(&MotionSensor, 2000);        // [-2000dps; +2000dps]
  LSM6DSOX_GYRO_Enable(&MotionSensor);                    // enable the gyroscope
}

int _write(int fd, char * ptr, int len) {

  HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
