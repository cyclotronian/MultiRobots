#include "stm32f10x.h"
#include "halDriver.h"
#include "vTasks.h"
#include "apps.h"

#include "FreeRTOS.h"
#include "task.h"
#define MULTIMELD
int main(void) {
  //////////////////////////////////////////////////////////////////////////////
  //1. hardware init
  //////////////////////////////////////////////////////////////////////////////
  SystemInit();    //STM32 cpu clock init -> 72MHZ
  halRCCInit();    //STM32 PeriphClock init
  //////////////////////////////////////////////////////////////////////////////
  //2. Periph devices init
  /////////////////////    //a. Led init
  WheelSpeedSensorInit();  //b. motor feedback speed sensor init 
  MotorInit();         /////////////////////////////////////////////////////////
  LedInit();               //c. motor driver init
  UartInit();              //d. uart Init, communicate with CC2530. ***it must be initialized, even uart is not used!!!***
  halMCUWaitMs(100);
  MPUSensorInit();         //e. MPU sensor Init -> gyro accel mag sensor
  RFInit();                //f. RF Init. via UART communicate with CC2530, config RF
  LightSensorInit();       //g. Light Sensor Init
  BeepInit();
  
  
  //建立任务
  //xTaskCreate( vMotorControlTask,    ( signed portCHAR * ) "CONTROL",  configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, NULL );
  extern void interruptMotorTask( void *pvParameters );
  xTaskCreate( interruptMotorTask,    ( signed portCHAR * ) "CONTROL",  configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, NULL );
//#ifdef MULTIMELD
  xTaskCreate( multiRobotVMTask,            ( signed portCHAR * ) "MULTIMELD",      configMINIMAL_STACK_SIZE*30, NULL, tskIDLE_PRIORITY+3, NULL );
//#endif 0

  //xTaskCreate( vReadMPUTask,       ( signed portCHAR * ) "MPU READ", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+7, NULL );
  //xTaskCreate( vBCastInfoTask,       ( signed portCHAR * ) "BCast",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, NULL );
  //xTaskCreate( vRFRxInfoTask,        ( signed portCHAR * ) "RF RX",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, NULL );
  
  //启动OS
  vTaskStartScheduler();
  
  return 0;
}

