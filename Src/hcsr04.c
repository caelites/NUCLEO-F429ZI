/**
  ******************************************************************************
  * @file           : hcsr04.c
  * @brief          : HC_SR04 Ultrasonic body
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "hcsr04.h"
#include "stm32f4xx_hal_tim.h"
    
/* Private variables ---------------------------------------------------------*/
// Flag
__IO ITStatus FLAG_Hcsr04_OnOff = SET;

/* Functions -----------------------------------------------------------------*/
// Moudle ON OFF
void HCSR04_T_Sonic_onoff(__IO ITStatus select_on_off){
    if(select_on_off == SET){
      FLAG_Hcsr04_OnOff = SET;
      HCSR04_T_Sonic_active();
    }
    else if(select_on_off == RESET)
      FLAG_Hcsr04_OnOff = RESET;
}

void HCSR04_T_Init(hcsr04_t* HCSR04, GPIO_TypeDef* ECHO_GPIOx, uint16_t ECHO_GPIO_Pin, GPIO_TypeDef* TRIG_GPIOx, uint16_t TRIG_GPIO_Pin, TIM_HandleTypeDef* HCSR_Timer){
    HCSR04->ECHO_GPIOx     = ECHO_GPIOx;
    HCSR04->ECHO_GPIO_Pin  = ECHO_GPIO_Pin;
    HCSR04->TRIG_GPIOx     = TRIG_GPIOx;
    HCSR04->TRIG_GPIO_Pin  = TRIG_GPIO_Pin;
    HCSR04->HCSR_Timer     = HCSR_Timer;
    HCSR04->Distance       = 0;
    
    // First detect
    HCSR04_T_Sonic_Active(TRIG_GPIOx, TRIG_GPIO_Pin);
}

// Active Trig
void HCSR04_T_Sonic_Active(GPIO_TypeDef* TRIG_GPIOx, uint16_t TRIG_GPIO_Pin){
  if(FLAG_Hcsr04_OnOff == SET){
     HAL_GPIO_WritePin(TRIG_GPIOx, TRIG_GPIO_Pin, SET);
     HAL_Delay(10);
     HAL_GPIO_WritePin(TRIG_GPIOx, TRIG_GPIO_Pin, RESET);
  }
}

// Measure distance
hcsr04_t HCSR04_T_Sonic_Measure(hcsr04_t hcsr04_echo, __IO ITStatus FLAG_Hcsr04, __IO ITStatus select_cm_or_inch){
    if(FLAG_Hcsr04 == RESET){
        __HAL_TIM_SetCounter(hcsr04_echo.HCSR_Timer,0);  // Init cnt
        HAL_TIM_Base_Start_IT(hcsr04_echo.HCSR_Timer);    // Start measure
    }
    else if(FLAG_Hcsr04 == SET){
        HAL_TIM_Base_Stop_IT(hcsr04_echo.HCSR_Timer);      // Stop measure
        
        // tmp measure data
        int measure_distance = __HAL_TIM_GetCounter(hcsr04_echo.HCSR_Timer); // Get measure timer counter
        
        // Selct cm or inch
        if(select_cm_or_inch == SET)
          hcsr04_echo.Distance = measure_distance/58;     // cm
        else
          hcsr04_echo.Distance = measure_distance/148;     // inch
        
        return hcsr04_echo;
    }
}


