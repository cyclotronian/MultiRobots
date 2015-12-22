#include "controlMotor.h"

void MotorInit(void) {
  halPWMInit();
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
  MotorRunEnable(false);
}

void MotorRunEnable(bool value){
  EnableMotorRun(value);
}

static typeWheelSpeed givenSpeed;

typeWheelSpeed GetWheelGivenSpeed(void) {
  return givenSpeed;
}
float GetLeftWheelGivenSpeed(void){
  return givenSpeed.left;
}
float GetRightWheelGivenSpeed(void){
  return givenSpeed.right;
}
void SetLeftWheelGivenSpeed(float speed){
  givenSpeed.left   = speed;
}
void SetRightWheelGivenSpeed(float speed){
  givenSpeed.right  = speed;
}
void SetWheelGivenSpeed(typeWheelSpeed speed){
  givenSpeed.left  = speed.left;
  givenSpeed.right = speed.right;
}

static float eL, eL_1, eL_2, uL, uL_1;
static float eR, eR_1, eR_2, uR, uR_1;
void ControlWheelSpeedAlg(typeWheelSpeed givenWheelSpeed, typeWheelSpeed feedbackWheelSpeed){
  float kP,kI,kD;
  float deltauL, deltauR;
  static float leftCtrlResult, rightCtrlResult;
  
  kP = PID_KP;
  kD = PID_KD;
  
  eL = givenWheelSpeed.left-feedbackWheelSpeed.left;       
  if (FloatAbs(eL)<5) {
    kI= PID_KI_3;
  }
  else if (FloatAbs(eL)<10) {
    kI = PID_KI_2;
  }
  else {
    kI = PID_KI_1;
  }

  deltauL = kP*(eL_1-eL_2) + kI*eL  + kD*(eL-2*eL_1+eL_2);
  uL = uL_1 + deltauL;
  uL_1 = uL;
  eL_2 = eL_1;
  eL_1 = eL;

  eR = givenWheelSpeed.right-feedbackWheelSpeed.right;
  if (FloatAbs(eR)<5) {
    kI= PID_KI_3;
  }
  else if (FloatAbs(eR)<10) {
    kI = PID_KI_2;
  }
  else {
    kI = PID_KI_1;
  }
  kP = PID_KP;
  kD = PID_KD;
  deltauR = kP*(eR_1-eR_2) + kI*eR  + kD*(eR-2*eR_1+eR_2);
  uR = uR_1 + deltauR;
  uR_1 = uR;
  eR_2 = eR_1;
  eR_1 = eR;

  leftCtrlResult = uL;
  rightCtrlResult= uR;
  //注意：左轮的转动方向是反的...需要处理...
  leftCtrlResult += 0.5;
  if (leftCtrlResult>1){
    leftCtrlResult = 1;
  }
  else if (leftCtrlResult<0) {
    leftCtrlResult = 0;
  }
  leftCtrlResult = 1-leftCtrlResult;
  
  rightCtrlResult += 0.5;
  if (rightCtrlResult>1) {
    rightCtrlResult = 1;
  }
  else if (rightCtrlResult<0){
    rightCtrlResult = 0;
  }
  SetMotorPWM(leftCtrlResult, rightCtrlResult);
}




