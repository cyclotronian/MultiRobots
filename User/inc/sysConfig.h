#ifndef __SYSCONFIG_H
#define __SYSCONFIG_H


//本节点ID: 应该在CC2530里面设置，为便便程序修改在STM32中设置
#define MY_NODE_ID                      (1)


//MOTOR CONTROL PERIOD && WHEEL SPEED READ PERIOD
#define MODULE_SPEED_CONTROL_PERIOD     (10)


//1. 车轮直径   19.5mm 19.7mm
#define WHEEL_DIAMETER                  (19.5)
//2. 车轮间距   61mm
#define WHEEL_L_R_DISTANCE              (61.2)
//3. 电机内置齿轮减速比1：28
#define MOTOR_GEAR_RATIO                (0.03571)
//4. 齿轮减速比 12:30
#define WHEEL_GEAR_RATIO                (0.4)
//5. PI=3.14159
#define PI                              (3.14159)
//6. 电机反馈线数 512 
#define MOTOR_FEEDBACK_TICKS            (512)

//MPU采集9轴的周期
#define MPU_SAMPLE_PERIOD               (20)

//ZigBee广播信息周期
#define BCAST_PERIOD                    (10)

//RF检查接收解码周期  
#define RF_DECODE_PERIOD                (10)

#endif 

