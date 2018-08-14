# NUCLEO-F429ZI
STM32F429ZI(NUCLEO-F429ZI 보드) HAL driver 를 이용한 RC차량

사용한 개발환경
   STM32CubeMX
	 IAR Embedded workbench

사용한 모듈
  초음파센서: HC-SR04
  모터 드라이버: L298N
  조도센서: CDS 센서
  블루투스 모듈: HC-06
  모션센서: MPU9250
  
라이브러리 진행된 상황
  HC-SR04 ---------- 완료
  L298N ------------ 10%
  CDS센서 ---------- 70%
  HC-06 ------------ 0%
  MPU9250 ---------- 50%(MPU6050으로 가능한 부분만 가능)
	
차량의 각 명령어는 main.c HAL_UART_RxCpltCallback 부분에 있고,
HC-SR04의 사용법은 HAL_GPIO_EXTI_Callback 부분을 참조

사용했던 핀 배열은 Pin.txt를 참조할것
