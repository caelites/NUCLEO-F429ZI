
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "CDSsensor_hal.h"
#include "hcsr04.h"
#include "L298N.h"
#include "mpu9250.h"
#include "Bluetooth_USART.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* L298N PWM pulse */
#define MOTOR_CCR_FULL 1000 // Full power
#define MOTOR_STOP 0        // Stop

/* car mode PWM value*/
uint16_t mode_d;        // Drive mode
uint16_t mode_r;        // Reverse mode

/**
 * @brief Power control calculate
 *        MOTOR_POWER = (mode_d/steer_rotate)/power_control;
**/
uint16_t steer_rotate_L = 1; // LEFT   
uint16_t steer_rotate_R = 1; // RIGHT
uint16_t power_control  = 4; // Power
uint8_t power_level = 1; // Power leve

/**
 * @brief Current car gear
 *        P - Parking
 *        D - Drive
 *        R - Reverse
**/
char car_gear_state = 'P';           

/* HC_SR04 */
hcsr04_t hcsr04;
uint32_t hcsr04_distance;
__IO ITStatus hcsr04_cm_or_inch = SET;  /* select measure value, return cm or inch */
__IO ITStatus hcsr04_on_off = SET;      /* turn on/off hcsr04 sensor */

/* USART2 Bluetooth */
char Rx2Data[2], Rx2Buf[2];
char Tx2Buf[512], Tx2Data[18] = "Wrong command \n";

/* I2C gyro sensor */
SD_MPU9250 mpu1;
int16_t I2CData[4];

/**
 * @brief Gyro module
 * Gyro X: Roll - R
 * Gyro Y: Pitch - P
 * Gyro Z: Yaw - Y
**/

/* FLAG */
// HCSR04
__IO ITStatus FLAG_Hcsr04 = RESET;
__IO ITStatus FLAG_Detect_Obstruction = RESET;

// Bluetooth UART
__IO ITStatus FLAG_RxCplt2 = RESET;

// Car 
__IO ITStatus FLAG_Accel = RESET;
__IO ITStatus FLAG_Break = RESET;
__IO ITStatus FLAG_Light = RESET;
__IO ITStatus FLAG_Light_Auto = SET;
__IO ITStatus FLAG_Light_State = RESET;

// Car remote auto mode
__IO ITStatus FLAG_Mode_Steer = RESET;
__IO ITStatus FLAG_Search_RL = SET; // SET = L , RESET = R

/** 
 * @brief Module state
 *        SET is working
 *        RESET is not working
**/
__IO ITStatus MODULE_hcsr04 = RESET;
__IO ITStatus MODULE_MPU    = RESET;
__IO ITStatus MODULE_CDS    = RESET;
__IO ITStatus MODULE_hc06   = RESET;
__IO ITStatus MODULE_L298N  = RESET;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int cds_light;
  uint8_t cds_cnt = 0;
  uint8_t trans_cnt = 0;
  
  SD_MPU9250_Result result ;
  uint8_t mpu_ok[15] = {"MPU WORK FINE\n"};
  uint8_t mpu_not[17] = {"MPU NOT WORKING\n"};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    
  // CDS Init
  // @brief 
  HAL_GPIO_WritePin(CDS_LED_GPIO_Port, CDS_LED_Pin, SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(CDS_LED_GPIO_Port, CDS_LED_Pin, RESET);
  
  // HCSR04 Init
  HCSR04_T_Init(&hcsr04, HCSR_ECHO_GPIO_Port, HCSR_ECHO_Pin, HCSR_TRIG_GPIO_Port, HCSR_TRIG_Pin, &htim7);
  
  // UART Init
  HAL_UART_Receive_IT(&huart2, (uint8_t*)Rx2Data,1);

  // L298N Init
  // Left motor
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_2);

  // Right motor
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_4);
  
  // Init PWM pulse
  // State Stop
  TIM2->CCR1 = 0;
  TIM2->CCR2 = 0;
  TIM2->CCR3 = 0;
  TIM2->CCR4 = 0;
  
  // Standard OK signal
  HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin, SET);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {      
    // Detects obsturction    
    if(hcsr04.Distance <= 15){
      if(hcsr04_on_off == SET){
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET);
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, RESET);
        FLAG_Detect_Obstruction = SET;
      }
    }      
    else{
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);
      HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, SET);
      FLAG_Detect_Obstruction = RESET;
    }
    
    // Car remote steer wheel Mode on/off state LED
    if(FLAG_Mode_Steer == SET)  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, SET);
    else HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, RESET);
                    
    // CDS Measure 5 times and divide avg
    if(FLAG_Light == RESET && FLAG_Light_Auto == SET){
      if(cds_cnt >= 5){
        cds_light = cds_light/5;
        if(cds_light <= 120){
          HAL_GPIO_WritePin(CDS_LED_GPIO_Port, CDS_LED_Pin, SET);
          FLAG_Light_State = SET; // State return to android application
        }
        else{
          HAL_GPIO_WritePin(CDS_LED_GPIO_Port, CDS_LED_Pin, RESET);
          FLAG_Light_State = RESET; // State return to android application
        }
        cds_light = 0;
        cds_cnt = 0;
      }
      else{
        cds_light += HAL_Measure_light(hadc1);
        cds_cnt++;
      }
    } else if(FLAG_Light == SET) {      // manual LE D ON
      HAL_GPIO_WritePin(CDS_LED_GPIO_Port, CDS_LED_Pin, SET);
      FLAG_Light_State = SET;
    } else {                            // manual LED OFF
      HAL_GPIO_WritePin(CDS_LED_GPIO_Port, CDS_LED_Pin, RESET);
      FLAG_Light_State = RESET; 
    }
        
    // Gear Set
    if(car_gear_state == 'P'){
      mode_d = MOTOR_STOP;
      mode_r = MOTOR_STOP;
    } else if(car_gear_state == 'D'){
      mode_d = MOTOR_CCR_FULL;
      mode_r = MOTOR_STOP;
    } else if(car_gear_state == 'R'){
      mode_d = MOTOR_STOP;
      mode_r = MOTOR_CCR_FULL;
    }
    
    // Detect obsturction
    if((FLAG_Detect_Obstruction == SET && car_gear_state != 'R')||car_gear_state == 'P'){
      if(TIM2->CCR1 != MOTOR_CCR_FULL && TIM2->CCR3 != MOTOR_CCR_FULL){
        TIM2->CCR1 = MOTOR_STOP;
        TIM2->CCR3 = MOTOR_STOP;
      }
      TIM2->CCR2 = MOTOR_STOP;
      TIM2->CCR4 = MOTOR_STOP;
    } else if(FLAG_Accel == SET){
      // Left motor
      TIM2->CCR2 = (mode_d/steer_rotate_L)/power_control;
      TIM2->CCR1 = (mode_r/steer_rotate_L)/power_control;
      // Right motor
      TIM2->CCR4 = (mode_d/steer_rotate_R)/power_control;
      TIM2->CCR3 = (mode_r/steer_rotate_R)/power_control;
    }
    
    // MPU Init
    result = SD_MPU9250_Init(&hi2c1,&mpu1,SD_MPU9250_Device_0,SD_MPU9250_Accelerometer_2G, SD_MPU9250_Gyroscope_250s);
    HAL_Delay(10);

    if(result == SD_MPU9250_Result_Ok) {
      if(SD_MPU9250_ReadGyroscope(&hi2c1, &mpu1) == SD_MPU9250_Result_Ok){
        I2CData[0] = mpu1.Gyroscope_X;
        I2CData[1] = mpu1.Gyroscope_Y;
        I2CData[2] = mpu1.Gyroscope_Z;
        //HAL_UART_Transmit_IT(&huart2,Tx2Buf,sprintf(Tx2Buf, "MPU Roll: %d Pitch: %d YAW: %d \n EOF1 ",I2CData[0] ,I2CData[1] ,I2CData[2]));
        HAL_UART_Transmit_IT(&huart2,Tx2Buf,sprintf(Tx2Buf, "1.MPU Roll: %d Pitch: %d YAW: %d \n 2.Gear: %c \n 3.LED: %s \n 4.HCSR: %s \n 5.DIS: %s \n %d \n",I2CData[0] ,I2CData[1] ,I2CData[2] ,car_gear_state ,FLAG_Light_State ? "ON" : "OFF" ,hcsr04_on_off ? "ON" : "OFF", FLAG_Detect_Obstruction ? "STOP" : "Go", power_level));
        HAL_Delay(50);
      }
    }
    else { 
        HAL_UART_Transmit_IT(&huart2,Tx2Buf,sprintf(Tx2Buf, "1.MPU Error Not working \n 2.Gear: %c \n 3.LED: %s \n 4.HCSR: %s \n 5.DIS: %s \n %d \n",car_gear_state ,FLAG_Light_State ? "ON" : "OFF" ,hcsr04_on_off ? "ON" : "OFF", FLAG_Detect_Obstruction ? "STOP" : "Go", power_level));
    }
    
    HAL_Delay(100); // delay 0.1s
  /* USER CODE END WHILE */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 116;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 58;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 58;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 58;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 23200;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_YELLOW_Pin|LED_RED_Pin|CDS_LED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HCSR_TRIG_GPIO_Port, HCSR_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MPU_RESET_GPIO_Port, MPU_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HCSR_ONOFF_Pin */
  GPIO_InitStruct.Pin = HCSR_ONOFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(HCSR_ONOFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HCSR_ECHO_Pin */
  GPIO_InitStruct.Pin = HCSR_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HCSR_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_YELLOW_Pin LED_RED_Pin CDS_LED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin|LED_RED_Pin|CDS_LED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HCSR_TRIG_Pin */
  GPIO_InitStruct.Pin = HCSR_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(HCSR_TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_RESET_Pin */
  GPIO_InitStruct.Pin = MPU_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MPU_RESET_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if(GPIO_Pin == HCSR_ECHO_Pin){
    if(FLAG_Hcsr04 == RESET){
      HCSR04_T_Sonic_Measure(hcsr04,FLAG_Hcsr04,hcsr04_cm_or_inch);
      FLAG_Hcsr04 = SET;
    }
    else if(FLAG_Hcsr04 == SET){
      hcsr04 = HCSR04_T_Sonic_Measure(hcsr04,FLAG_Hcsr04,hcsr04_cm_or_inch);
      FLAG_Hcsr04 = RESET;
      HCSR04_T_Sonic_Active(hcsr04.TRIG_GPIOx, hcsr04.TRIG_GPIO_Pin);
    }
  }
}

/**
  * @brief  UART Rx interrupt Callback.
  * @param  huart: used to uart serial handle
  * @retval None
  * @deprecated Refactor library
  * @TODO convert library
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART2){
    for(int i=0;i<2;i++)
      Rx2Buf[i] = Rx2Data[i];

    switch(Rx2Buf[0]){
      // power control
      case 'A':
        if(car_gear_state != 'P'){
          if(FLAG_Mode_Steer == SET && FLAG_Detect_Obstruction == RESET){
            if(FLAG_Detect_Obstruction != SET || car_gear_state == 'R'){
              FLAG_Accel = SET;
            } else {
              HAL_UART_Transmit_IT(&huart2, "There is an obstacle in front.\n", 31);
            }
          } else if(FLAG_Detect_Obstruction == RESET){
            TIM2->CCR1 = MOTOR_STOP;
            TIM2->CCR2 = MOTOR_CCR_FULL/power_control;
            TIM2->CCR3 = MOTOR_STOP;
            TIM2->CCR4 = MOTOR_CCR_FULL/power_control;
          }
        }
        break;
     
     case 'B':
       // Left motor
       TIM2->CCR1 = MOTOR_STOP;
       TIM2->CCR2 = MOTOR_STOP;
       // Right motor
       TIM2->CCR3 = MOTOR_STOP;
       TIM2->CCR4 = MOTOR_STOP;
       // Accel
       FLAG_Accel = RESET;
       break; 
          
     // steer wheel control
     // Rigth
     case 'E':
       if(car_gear_state != 'P'){
         if(FLAG_Mode_Steer == SET && FLAG_Detect_Obstruction == RESET){
           if(steer_rotate_R <= 3 && steer_rotate_L == 1)
              steer_rotate_R++;
           else if(steer_rotate_L > 1)
              steer_rotate_L--;
         } else if(FLAG_Detect_Obstruction == RESET){
           // Left motor
           TIM2->CCR1 = MOTOR_STOP;
           TIM2->CCR2 = MOTOR_CCR_FULL/power_control;
           // Right motor
           TIM2->CCR3 = MOTOR_STOP;
           TIM2->CCR4 = MOTOR_STOP;
         }
       }
       break;
     
     // Left
     case 'Q':
       if(car_gear_state != 'P'){
         if(FLAG_Mode_Steer == SET && FLAG_Detect_Obstruction == RESET){
           if(steer_rotate_L <= 3 && steer_rotate_R == 1)
              steer_rotate_L++;
           else if(steer_rotate_R > 1)
              steer_rotate_R--;
         } else if(FLAG_Detect_Obstruction == RESET){
           // Left motor
           TIM2->CCR1 = MOTOR_STOP;
           TIM2->CCR2 = MOTOR_STOP;
           // Right motor
           TIM2->CCR3 = MOTOR_STOP;
           TIM2->CCR4 = MOTOR_CCR_FULL/power_control;
         }
       }
       break; 
     
      // BACK
      case 'S':
        if(FLAG_Mode_Steer == RESET && car_gear_state != 'P'){
          TIM2->CCR1 = MOTOR_CCR_FULL/power_control;
          TIM2->CCR2 = MOTOR_STOP;
          TIM2->CCR3 = MOTOR_CCR_FULL/power_control;
          TIM2->CCR4 = MOTOR_STOP;
        } 
        break; 
       
     // Gear control
     case 'P':
       car_gear_state = 'P';
       break;
       
     case 'D':
       car_gear_state = 'D';
       break;
     
     case 'R':
       if(FLAG_Mode_Steer == SET){
         car_gear_state = 'R';
       }
       break;
       
     // Power Control
     case '3':
       power_control = 1;
       power_level = 3;
       break;
       
     case '2':
       power_control = 2;
       power_level = 2;
       break;
       
     case '1':
       power_control = 4;
       power_level = 1;
       break;
     
     // Mode select
     case 'Z':
       FLAG_Mode_Steer = !FLAG_Mode_Steer;
       switch(FLAG_Mode_Steer){
          case SET:
          break;
          
          case RESET:
          break;
       }
       break;
        
     // LED Light on off
     case 'L':
       FLAG_Light = !FLAG_Light;
       break;
     
     // HCSR04_on_off
     case 'H':
       hcsr04_on_off = !hcsr04_on_off;
       break;
         
     default:
      HAL_UART_Transmit_IT(&huart2, (uint8_t*)Tx2Data, 18);
      break;
    }
    HAL_UART_Receive_IT(&huart2, (uint8_t*)Rx2Data, 1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
