# NUCLEO-F429ZI<br>
STM32F429ZI(NUCLEO-F429ZI 보드) HAL driver 를 이용한 RC차량<br><br>

사용한 개발환경<br>
   STM32CubeMX<br>
   IAR Embedded workbench<br><br>

사용한 모듈<br>
  초음파센서: HC-SR04<br>
  모터 드라이버: L298N<br>
  조도센서: CDS 센서<br>
  블루투스 모듈: HC-06<br>
  모션센서: MPU9250<br>
  <br>
라이브러리 진행된 상황<br>
  HC-SR04 ---------- 완료<br>
  L298N ------------ 10%<br>
  CDS센서 ---------- 70%<br>
  HC-06 ------------ 0%<br>
  MPU9250 ---------- 50%(MPU6050으로 가능한 부분만 가능)<br>
	<br>
차량의 각 명령어는 main.c HAL_UART_RxCpltCallback 부분에 있고,<br>
HC-SR04의 사용법은 HAL_GPIO_EXTI_Callback 부분을 참조<br><br>

사용했던 핀 배열은 Pins.txt를 참조할것


