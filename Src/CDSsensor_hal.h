/**
   README 
   @file: CDSsensor_hal.h
   @breif CDS light measure sensor
   Model: CDS sensor
   Details: This sensor can measure light. Sensor get analog data to convert digital data. 
**/

#ifndef _CDS_SENSOR_H_
#define _CDS_SENSOR_H_

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
   
/* Set functions -------------------------------------------------------------*/
/**
 * @brief  Measure light sensor
 * @param  None
 * @return int
**/
int HAL_Measure_light();

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _CDS_SENSOR_H_ */