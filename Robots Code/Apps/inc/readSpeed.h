#ifndef __READSPEED_H
#define __READSPEED_H

#include "FreeRTOS.h"
#include "task.h"

#include "halABDecoder.h"
#include "sysConfig.h"
#include "stm32f10x.h"

////////////////////////////////////////////////////////////////////////////////////
//��������ת����ز���
////////////////////////////////////////////////////////////////////////////////////
//1. ����ֱ��   19.5mm 19.7mm
#define WHEEL_DIAMETER                  (19.5)
//2. ���ּ��   61mm
#define WHEEL_L_R_DISTANCE              (61.2)
//3. ������ó��ּ��ٱ�1��28
#define MOTOR_GEAR_RATIO                (0.03571)
//4. ģ�ͳ��ּ��ٱ� 12:30
#define WHEEL_GEAR_RATIO                (0.4)
//5. PI=3.14159
#define PI                              (3.14159)
//6. ����������� 512  �������ÿתһȦ����������512
#define MOTOR_FEEDBACK_TICKS            (512)
 
////////////////////////////////////////////////////////////////////////////////////
//speed unit    �� mm/second
//direction unit:  degree
////////////////////////////////////////////////////////////////////////////////////
typedef struct typeWheelSpeed{
  float left;       
  float right;
}typeWheelSpeed;
 
typeWheelSpeed ReadWheelSpeed(void);
extern void WheelSpeedSensorInit(void);
extern typeWheelSpeed GetWheelSpeed(void);


#endif

