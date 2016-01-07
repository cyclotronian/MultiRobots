#ifndef __VBCASTINFOTASK_H
#define __VBCASTINFOTASK_H


#include "FreeRTOS.h"
#include "task.h"
#include "halLed.h"
#include "halUsart.h"
#include "sysConfig.h"
#include "string.h"

//ָ���
#define USART_SET_ID             (1)
#define USART_RF_SEND_DATA       (2)
#define USART_RF_REC_DATA        (3)
#define USART_USONIC_DIS2B1      (4)
#define USART_USONIC_DIS2B2      (5)

//����26
typedef struct typeBcastInfo{
  float   coordinateX; //
  float   coordinateY;
  float   speedL;      //�����ٶ�
  float   speedR;      //�����ٶ� 
  float   angle2n;     //�Ա��Ƕ�
}typeBcastInfo;

#include "halMPUInit.h"
#include "motorFunction.h"

extern void vBCastInfoTask( void *pvParameters );

#endif
