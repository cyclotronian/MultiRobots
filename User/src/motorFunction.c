#include "motorFunction.h"

////////////////////////////////////////////////////////////////////////////////
//反馈速度相关函数
////////////////////////////////////////////////////////////////////////////////
static typeWheelSpeed wheelSpeed;
void CalculateWheelSpeed(signed long xValue, signed long yValue, u32 timeDiff){
  wheelSpeed.left  = ((float)xValue/(float)MOTOR_FEEDBACK_TICKS)*(1000.0/(float)timeDiff)*WHEEL_DIAMETER*PI*MOTOR_GEAR_RATIO*WHEEL_GEAR_RATIO;
  wheelSpeed.right = ((float)yValue/(float)MOTOR_FEEDBACK_TICKS)*(1000.0/(float)timeDiff)*WHEEL_DIAMETER*PI*MOTOR_GEAR_RATIO*WHEEL_GEAR_RATIO;  
}

typeWheelSpeed GetWheelSpeed(void){
  return wheelSpeed;
}

float GetWheelLSpeed(void) {
  return wheelSpeed.left;
}

float GetWheelRSpeed(void) {
  return wheelSpeed.right;
}

////////////////////////////////////////////////////////////////////////////////
//电机速度控制相关函数
////////////////////////////////////////////////////////////////////////////////

static typeWheelSpeed wheelSpeedGiven;

typeWheelSpeed GetWheelGivenSpeed(void){
  return wheelSpeedGiven;
}
float GetWheelLGivenSpeed(void) {
  return wheelSpeedGiven.left;
}
float GetWheelRGivenSpeed(void) {
  return wheelSpeedGiven.right;
}
void SetWheelGivenSpeed(float valueL, float valueR) {
  wheelSpeedGiven.left = valueL;
  wheelSpeedGiven.right = valueR;
}
void SetWheelLGivenSpeed(float value){
  wheelSpeedGiven.left = value;
}
void SetWheelRGivenSpeed(float value){
  wheelSpeedGiven.right = value;
}

//------------------------------------------------------------------------------
//增量式PID 算法
//deltaU = a*e(k) - b*e(k-1) + c*e(k-2)
//a=Kp*(1+T/Ti+Td/T) 
//b=Kp*(1+2*Td/T)
//C=Kp*Td/T
//------------------------------------------------------------------------------
static float kP, kI, kD;
void PIDParaInit(void) {
  //kA = Kp*(1+KT/KTi+KTd/KT);
  //kB = Kp*(1+2*KTd/KT);
  //kC = Kp*(KTd/KT);
  //kA = PID_KP;
  //kB = PID_KI;
  //kC = PID_KD;
}

//电机两侧安装，模块前行必须一个正转，一个反转
//右边电机正转为向前，那么左边电机负转为向前，左边电机在内部计算中给定速度都娶负值
//
//Uk=Kp*ek + Ki*(e0+e1+.....+ek) + Kd(ek-ek_1)
//Uk = Uk_1 + deltaUk;
//deltaUk = Kp*(ek-ek_1) + Ki*ek + Kd*(ek-2*ek_1 + ek_2)
//
static float ux=0, ux_1=0, ex=0, ex_1=0, ex_2=0, deltaUx=0;
float PIDControlLMotor(void){     //给定速度单位mm/sec
  ex = 0-wheelSpeedGiven.left-wheelSpeed.left;
  
  kP = PID_KP;
  kD = PID_KD;
  
  if (fabs(ex)<5) {
    kI= PID_KI_3;
  }
  else if (fabs(ex)<10) {
    kI = PID_KI_2;
  }
  else {
    kI = PID_KI_1;
  }
  
  deltaUx = kP*(ex_1-ex_2) + kI*ex  + kD*(ex-2*ex_1+ex_2);
  ux = ux_1 + deltaUx;
  ux_1 = ux;
  ex_2 = ex_1;
  ex_1 = ex;
  return ux;
}

static float uy=0, uy_1=0, ey=0, ey_1=0, ey_2=0, deltaUy=0;
float PIDControlRMotor(void){    //给定速度单位mm/sec
  ey = wheelSpeedGiven.right-wheelSpeed.right;

  kP = PID_KP;
  kD = PID_KD;
  
  
  
  if (fabs(ex)<5) {
    kI= PID_KI_3;
  }
  else if (fabs(ex)<10) {
    kI = PID_KI_2;
  }
  else {
    kI = PID_KI_1;
  }
  
  deltaUy = kP*(ey_1-ey_2) + kI*ey  + kD*(ey-2*ey_1+ey_2);
  uy = uy_1 + deltaUy;
  uy_1 = uy;
  ey_2 = ey_1;
  ey_1 = ey;
  return uy;
}

float CalFloatAbs(float data){
  if (data>0) {
    return data;
  }
  return (0-data);
}

////////////////////////////////////////////////////////////////////////////////
//原地旋转角度
//angle->度, speed->毫米/秒
////////////////////////////////////////////////////////////////////////////////
void ModuleTurnAngle(float speed, float angle){
  static u32 expectPeriod;
  expectPeriod=(u32)((PI*WHEEL_L_R_DISTANCE*1000 / speed) * (angle/360.0f));
  SetWheelLGivenSpeed(speed);
  SetWheelRGivenSpeed(0-speed);
  vTaskDelay(expectPeriod);
  SetWheelLGivenSpeed(0);
  SetWheelRGivenSpeed(0);
}

void ModuleGoStrait(float speed, float dis){
  static u32 expectPeriod;
  expectPeriod=(u32)(dis*1000/speed);
  SetWheelLGivenSpeed(speed);
  SetWheelRGivenSpeed(speed);
  vTaskDelay(expectPeriod);
  SetWheelLGivenSpeed(0);
  SetWheelRGivenSpeed(0);
}


