#include "readSpeed.h"
 

typeWheelSpeed weelFeedbackSpeed;
static u32 lastReadSpeedTime=0;

void WheelSpeedSensorInit(void) {
  halABDecoderInit();
}

typeWheelSpeed GetWheelSpeed(void) {
  return weelFeedbackSpeed;
}

////////////////////////////////////////////////////////////////////////////////
//��ȡģ���������ֵ��ٶ�
////////////////////////////////////////////////////////////////////////////////
typeWheelSpeed ReadWheelSpeed(void){
  typeWheelSpeed wheelSpeedTemp;
  static signed long readXValue, readYValue;
  u32 xTimeDiff, currentReadSpeedTime;
  u32 xNowTimeTick;
  xNowTimeTick = xTaskGetTickCount();
  if ((xNowTimeTick - lastReadSpeedTime)<10){    //max read freq=100HZ
    return weelFeedbackSpeed;
  }
  readXValue = halABXDecoderReadData();
  halABClearABXInhibitLogic();
  readYValue = halABYDecoderReadData();
  halABClearABYInhibitLogic();
  currentReadSpeedTime = xTaskGetTickCount();
  xTimeDiff = currentReadSpeedTime-lastReadSpeedTime;
  lastReadSpeedTime = currentReadSpeedTime;
  //��Ϊ�������������෴��װ������ȷ��ǰ������
  wheelSpeedTemp.right  = ((float)readXValue/(float)MOTOR_FEEDBACK_TICKS)*(1000.0/(float)xTimeDiff)*WHEEL_DIAMETER*PI*MOTOR_GEAR_RATIO*WHEEL_GEAR_RATIO;
  wheelSpeedTemp.left   = ((float)readYValue/(float)MOTOR_FEEDBACK_TICKS)*(1000.0/(float)xTimeDiff)*WHEEL_DIAMETER*PI*MOTOR_GEAR_RATIO*WHEEL_GEAR_RATIO;  
  weelFeedbackSpeed.left  = wheelSpeedTemp.left;
  weelFeedbackSpeed.right = wheelSpeedTemp.right;
  return wheelSpeedTemp;
}

