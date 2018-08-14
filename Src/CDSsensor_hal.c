/**
  ******************************************************************************
  * @file           : CDSsensor_hal.c
  * @brief          : CDS light sensor body
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "CDSsensor_hal.h"
#include "stm32f4xx_hal.h"

// Measure light
int HAL_Measure_light(ADC_HandleTypeDef hadc1){
    int light;

    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK){
      light = HAL_ADC_GetValue(&hadc1);
    }

    return light;
}