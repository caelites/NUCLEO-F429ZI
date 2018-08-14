/**
   README 
   @file: hcsr04.h
   @breif Ultrasonic dectect distance sensor
   Model: HC_SR04
   Details: レs/58 = cm, レs/148 = inch, レs(micro scecond) -> Measureed with an oscilloscope
            Timer clock must be set to 1 レs,
            example) your clock is 45Mhz, you must be set prescaler 45(Or, The real time setting of 1レs measureable value)
            clock and period max is maximum measureable value divide to prescler.
            The maximum value at which the hc_sr04 sensor can be measured is 2cm ~ 400cm (period 116 ~ 23,200).

   Directions: Trig turn on 10レs, and produces 8 sonic pulse. Then echo pin detects an interrupts(rising or falling edge).
               When the Echo pin is in the rising edge state, Turn on TIM, and when exit TIM the Echo pin falling edge.
**/

#ifndef __ULTRA_SONIC_HCSR04_H_
#define __ULTRA_SONIC_HCSR04_H_

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Private variables ---------------------------------------------------------*/
/**
 * @breif HC-SR04 sensor structure
**/   
typedef struct{
  int                Distance;        /* Distance measured from sensor in centi meters */
  GPIO_TypeDef*      ECHO_GPIOx;      /* Pointer to GPIOx PORT for ECHO pin. Meant for private use only */
  uint16_t           ECHO_GPIO_Pin;   /* GPIO Pin for ECHO pin. Meant for private use only */
  GPIO_TypeDef*      TRIG_GPIOx;      /* Pointer to GPIOx PORT for TRIGGER pin. Meant for private use only */
  uint16_t           TRIG_GPIO_Pin;   /* GPIO Pin for ECHO pin. Meant for private use only */
  TIM_HandleTypeDef* HCSR_Timer;      /* Timer for measurement when sensor is rising edge. */
}hcsr04_t;

/* Set functions -------------------------------------------------------------*/
/**
 * @brief  Sensor on off
 * @param  select_on_off: Sensor on off control
 * @see    select_on_off status:
 *            - SET: Device is ready to use
 *            - RESET: Device not detected
 */
void HCSR04_T_Sonic_onoff(__IO ITStatus select_on_off);

/**
 * @brief  Initializes HC-SR04 sensor
 * @param  *HCSR04: Pointer to empty @ref TM_HCSR04_t structure to save initialization data
 * @param  *ECHO_GPIOx: Pointer to GPIOx PORT for ECHO pin
 * @param  ECHO_GPIO_Pin: GPIO Pin for ECHO pin
 * @param  *TRIG_GPIOx: Pointer to GPIOx PORT for TRIGGER pin
 * @param  TRIG_GPIO_Pin: GPIO Pin for ECHO pin
 * @param  *HCSR_Timer: Pointer to TIM handler
 * @see    HC-SR04 status:
 *              - 0: Device not detected
 *              - > 0: Device is ready to use
 */
void HCSR04_T_Init(hcsr04_t* HCSR04, GPIO_TypeDef* ECHO_GPIOx, uint16_t ECHO_GPIO_Pin, GPIO_TypeDef* TRIG_GPIOx, uint16_t TRIG_GPIO_Pin, TIM_HandleTypeDef* HCSR_Timer); 

/**
 * @brief  Procedures ultrasonic
 * @param  hcsr04_trig: HCSR04 sensor pin for TIRG pin
 * @see    FLAG_Hcsr04_OnOff Status:
 *              - SET: working
 *              - RESET: not working
 */
void HCSR04_T_Sonic_Active(GPIO_TypeDef* TRIG_GPIOx, uint16_t TRIG_GPIO_Pin);

/**
 * @brief  Measure sensor
 * @param  hcsr04_echo: hcsr04_t structure
 * @param  select_cm_or_inch: cm or inch
 * @param  FLAG_Hcsr04: rising edge or falling edge
 * @see    select_cm_or_inch status:
 *              - SET: cm(レs/58)
 *              - RESET: inch(レs/148)
 *
 * @see    FLAG_Hcsr04 status:
 *              - SET: rising edge
 *              - RESET: falling edge
 */  
hcsr04_t HCSR04_T_Sonic_Measure(hcsr04_t hcsr04_echo, __IO ITStatus select_cm_or_inch, __IO ITStatus FLAG_Hcsr04);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ULTRA_SONIC_HCSR04_H */ 