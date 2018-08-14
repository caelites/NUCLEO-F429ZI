/**
   README 
   @file: Bluetooth_USART.h
   @breif UART serial bluetooth network
   Model: HC-06
   Details: 
**/

#ifndef _USART_BLUETOOTH_HC_04_H_
#define _USART_BLUETOOTH_HC_04_H_

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Set functions -------------------------------------------------------------*/   
void UART_BT_RxData();
void UART_BT_TxData();

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _USART_BLUETOOTH_HC_04_H_ */