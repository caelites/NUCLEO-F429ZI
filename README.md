# NUCLEO-F429ZI<br>
STM32F429ZI(NUCLEO-F429ZI 보드) HAL driver 를 이용한 RC차량<br><br>

사용한 개발환경<br>
&nbsp; &nbsp; STM32CubeMX<br>
&nbsp; &nbsp; IAR Embedded workbench ARM 8.30<br><br>

사용한 모듈<br>
&nbsp; &nbsp; 초음파센서: HC-SR04<br>
&nbsp; &nbsp; 모터 드라이버: L298N<br>
&nbsp; &nbsp; 조도센서: CDS 센서<br>
&nbsp; &nbsp; 블루투스 모듈: HC-06<br>
&nbsp; &nbsp; 모션센서: MPU9250<br>
  <br>

차량의 각 명령어는 main.c HAL_UART_RxCpltCallback 부분에 있고,<br>
HC-SR04의 사용법은 HAL_GPIO_EXTI_Callback 부분을 참조<br><br>

사용했던 핀 배열은 Pins.txt를 참조<br>
