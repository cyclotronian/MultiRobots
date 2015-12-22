#ifndef __VBCASTINFOTASK_H
#define __VBCASTINFOTASK_H


#include "FreeRTOS.h"
#include "task.h"
#include "halLed.h"
#include "halUsart.h"
#include "sysConfig.h"
#include "string.h"

//指令定义
#define USART_SET_ID             (1)
#define USART_RF_SEND_DATA       (2)
#define USART_RF_REC_DATA        (3)
#define USART_USONIC_DIS2B1      (4)
#define USART_USONIC_DIS2B2      (5)

//共计26
typedef struct typeBcastInfo{
  float   coordinateX; //
  float   coordinateY;
  float   speedL;      //左轮速度
  float   speedR;      //右轮速度 
  float   angle2n;     //对北角度
}typeBcastInfo;

#include "halMPUInit.h"
#include "motorFunction.h"

extern void vBCastInfoTask( void *pvParameters );

#endif
