#ifndef __SYSCONFIG_H
#define __SYSCONFIG_H


//���ڵ�ID: Ӧ����CC2530�������ã�Ϊ�������޸���STM32������
#define MY_NODE_ID                      (1)


//MOTOR CONTROL PERIOD && WHEEL SPEED READ PERIOD
#define MODULE_SPEED_CONTROL_PERIOD     (10)


//1. ����ֱ��   19.5mm 19.7mm
#define WHEEL_DIAMETER                  (19.5)
//2. ���ּ��   61mm
#define WHEEL_L_R_DISTANCE              (61.2)
//3. ������ó��ּ��ٱ�1��28
#define MOTOR_GEAR_RATIO                (0.03571)
//4. ���ּ��ٱ� 12:30
#define WHEEL_GEAR_RATIO                (0.4)
//5. PI=3.14159
#define PI                              (3.14159)
//6. ����������� 512 
#define MOTOR_FEEDBACK_TICKS            (512)

//MPU�ɼ�9�������
#define MPU_SAMPLE_PERIOD               (20)

//ZigBee�㲥��Ϣ����
#define BCAST_PERIOD                    (10)

//RF�����ս�������  
#define RF_DECODE_PERIOD                (10)

#endif 

