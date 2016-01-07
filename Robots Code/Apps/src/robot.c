#include "robot.h"
#include "vInfoList.h"
#include <math.h>

float ReadMagSensorAngle2North(void) {
  typeMagSensor magSensor;
  float edgeLong=0;
  float compX, compY;
  float angleReturn;
  
  magSensor =  ReadMagSensor();
  compX = magSensor.magX-MAG_SENSOR_X;
  compY = magSensor.magY-MAG_SENSOR_Y;

  edgeLong= sqrt(compX*compX + compY*compY);
  
  angleReturn = (180.0f/3.14) * (acos(compY/edgeLong));  
 
  if (compX<0){    
    angleReturn=0-angleReturn;
  }      
 
  return angleReturn;
}

////////////////////////////////////////////////////////////////////////////////
//calculate coordinate
////////////////////////////////////////////////////////////////////////////////
typeCoordinate GetCoordinate(void) {
  float disB1, disB2;
  static typeCoordinate coordinateC;
  
  disB1 = GetDistanse2B1();
  disB2 = GetDistanse2B2();
  
  coordinateC.y = (disB1*disB1 - disB2*disB2 + DISTANSE_B1_2_B2*DISTANSE_B1_2_B2)/(2*DISTANSE_B1_2_B2);
  coordinateC.x = sqrt(disB1*disB1 - DISTANSE_B1_2_B2*DISTANSE_B1_2_B2 - coordinateC.y*coordinateC.y);
  
  return coordinateC;
}

///////////////////////////////
/////////////////////////////////////////////////
//angle: range from -180 to 180 
////////////////////////////////////////////////////////////////////////////////
void ControlRobotRotate(float angle, float speed) {
  float givenAngle;
  static float timeInterval;
  if (FloatAbs(speed) > MAX_ROTATE_SPEED) {
    if (speed<0) {
      speed = 0 - MAX_ROTATE_SPEED;
    }
    else {
      speed = MAX_ROTATE_SPEED;
    }
  }
  if (angle<0) {
    givenAngle = -angle/180*PI;
    SetLeftWheelGivenSpeed(speed);
    SetRightWheelGivenSpeed(0-speed);
  }
  else {
    givenAngle = angle/180*PI;
    SetLeftWheelGivenSpeed(0-speed);
    SetRightWheelGivenSpeed(speed);
  }
  timeInterval = 1000*givenAngle*(WHEEL_L_R_DISTANCE)/(2*speed);
  vTaskDelay((unsigned long)timeInterval);
  
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
}

////////////////////////////////////////////////////////////////////////////////
//angle: control Robot go from current position to given position
////////////////////////////////////////////////////////////////////////////////
void ControlRobotFollowLine(float x0, float y0, float x1, float y1, float speed){
  
}
////////////////////////////////////////////////////////////////////////////////
//angle: control Robot go from current position to given position
////////////////////////////////////////////////////////////////////////////////
void ControlRobotFollowCircle(float x, float y, float speed){
  
}

////////////////////////////////////////////////////////////////////////////////
//angle: given two coordinates to calculate line dir
////////////////////////////////////////////////////////////////////////////////
float GetLineDirection(float x0, float y0, float x1, float y1) {
  float angle;
  float eLong;
  float comX, comY;
  
  comX = x1-x0;
  comY = y1-y0;
  
  eLong = sqrt(comX*comX + comY*comY);
  angle = (180/M_PI) * acos(FloatAbs(comX)/eLong);  //0 to 90 degree
  
  if ((comX>=0) && (comY>=0)) {     // 1
    angle = angle+90;
  }
  if ((comX<=0) && (comY>=0)) {     //2 
    angle = 90-angle-180;
  }
  if ((comX<=0) && (comY<=0)) {     //3
    angle = angle-90;
  }
  if ((comX>=0) && (comY<=0)) {     //4
    angle = 90 - angle;
  }
  return angle;
}


bool RobotInRange(float x0, float y0, float x1, float y1) {
  if ((FloatAbs(x1-x0)>0.03) || (FloatAbs(y1-y0)>0.03)){
    return false;
  }
  return true;
}

void northRotate(float x, float y, float speed){
  static typeCoordinate coordinate;
  float lineDir;
  float robotDir;
  
  //1. calculate coordinate
  coordinate = GetCoordinate();
  
  //2. calculate direction of line
  lineDir = GetLineDirection(coordinate.x, coordinate.y, x, y);
  //3. turn dir to line
  robotDir = ReadMagSensorAngle2North();
  
  if ((lineDir-robotDir)<-180) {
    ControlRobotRotate(lineDir-robotDir+360-20, 5);
  }
  else if ((lineDir-robotDir)>180) {
    ControlRobotRotate(lineDir-robotDir-360+20, 5);
  }
  else {
    ControlRobotRotate(lineDir-robotDir, 5);
  }
}

////////////////////////////////////////////////////////////////////////////////
//angle: control Robot go from current position to given position
////////////////////////////////////////////////////////////////////////////////
void ControlRobot2Position(float x, float y, float speed){
  static typeCoordinate coordinate;
  float speedL, speedR;
  //float calY;
  float lineDir;
  float robotDir;
  
  speedL = speed;
  speedR = speed;
  
  //1. calculate coordinate
  coordinate = GetCoordinate();
  //2. y=kx+b, calculate line para k and b
  float k, b;
  k = (y-coordinate.y)/(x-coordinate.x);
  b = y-k*x;
  //3. calculate direction of line
  lineDir = GetLineDirection(coordinate.x, coordinate.y, x, y);
  //4. turn dir to line
  robotDir = ReadMagSensorAngle2North();
  float turnangle = lineDir - robotDir;
  while (turnangle < 0) turnangle += 360;
  while (turnangle >= 360) turnangle -= 360;
  if (turnangle > 180) turnangle = 360 - turnangle;
  ControlRobotRotate(turnangle, 5);
  /*
  if ((lineDir-robotDir)<-180) {
    ControlRobotRotate(lineDir-robotDir+360-20, 5);
  }
  else if ((lineDir-robotDir)>180) {
    ControlRobotRotate(lineDir-robotDir-360+20, 5);
  }
  else {
    ControlRobotRotate(lineDir-robotDir, 5);
  }
  */

  SetLeftWheelGivenSpeed(1);
  SetRightWheelGivenSpeed(1);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(3);
  SetRightWheelGivenSpeed(3);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(5);
  SetRightWheelGivenSpeed(5);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(10);
  SetRightWheelGivenSpeed(10);
  vTaskDelay(300);
  SetLeftWheelGivenSpeed(15);
  SetRightWheelGivenSpeed(15); 
  vTaskDelay(300);
  SetLeftWheelGivenSpeed(20);
  SetRightWheelGivenSpeed(20);
  vTaskDelay(1000);
  
  coordinate = GetCoordinate();
  while (RobotInRange(coordinate.x,coordinate.y, x, y)==false){
    /*
    if (coordinate.y>(k*coordinate.x+b)) {
      speedL = 25;
      speedR = 20;      
    }
    else {
      speedL = 20;
      speedR = 25;
    }
    */
    if (coordinate.y>(k*coordinate.x+b)+0.01) {
      speedL = 23;
      speedR = 20;
    }
    else if (coordinate.y+0.0<(k*coordinate.x+b)) {
      speedL = 20;
      speedR = 23;
    }
    else {
      robotDir = ReadMagSensorAngle2North();
      if ((lineDir-robotDir)>20){
        speedL = 20;
        speedR = 23;
      }
      else if ((lineDir-robotDir)<-20){
        speedL = 23;
        speedR = 20;
      }
          
      else{
        speedL = 20;
        speedR = 20;
      }
    }
    
    SetLeftWheelGivenSpeed(speedL);
    SetRightWheelGivenSpeed(speedR);
    vTaskDelay(500);
    coordinate = GetCoordinate();
  }
  
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
  vTaskDelay(100000);

  asm("NOP");
}

  
////////////////////////////////////////////////////////////////////////////////
//1. check robot stability 
//2. get robot's direction
////////////////////////////////////////////////////////////////////////////////
void RobotPrepare(void) {
  //1. make robot stop
  typeWheelSpeed speed;
  typeMPUSensor mpuAccel;
  u8 retryTimes=0;
  static typeCoordinate startCoordinate, endCoordinate;
  float diffX, diffY;
  float angle;
  
  MotorRunEnable(true);
  speed.left  = 0;
  speed.right = 0;
  SetWheelGivenSpeed(speed);
  speed = GetWheelSpeed();
  while ((speed.left > 1) || (speed.right > 1)); {
    vTaskDelay(50);
    speed = GetWheelSpeed();
  }
  MotorRunEnable(false);
  //2. read MPU accel sensor, check robot's stalibility
  mpuAccel.accelX = 20;
  mpuAccel.accelY = 20;
  
  while ( retryTimes< ROBOT_STABLE_RETRY_THR ){
    mpuAccel = ReadMPUSensor();
    if ((FloatAbs(mpuAccel.accelX)<ROBOT_STABLE_ACCEL_X_THR) && (FloatAbs(mpuAccel.accelY)<ROBOT_STABLE_ACCEL_X_THR)){ 
      retryTimes++;
    }
    else {
      retryTimes = 0;
    }
    vTaskDelay(50);
  }
  //3. rotate to find x min compass 
  startCoordinate = GetCoordinate();
  
  MotorRunEnable(true);
  speed.left  = 10;
  speed.right = 10;
  SetWheelGivenSpeed(speed);
  vTaskDelay(4000);
  speed.left  = 0;
  speed.right = 0;
  SetWheelGivenSpeed(speed);
  
  endCoordinate = GetCoordinate();
  diffX = endCoordinate.x - startCoordinate.x;
  diffY = endCoordinate.y - startCoordinate.y;
  
  angle =(180/3.14)*acos(diffX / sqrt(diffY*diffY + diffX*diffX));
  
  if (diffY < 0) {
    angle = 0-angle;
  }
  SetRobotAngle(angle);
  
  //4. 
}

float robotAngle=0;
void SetRobotAngle(float angle) {
  robotAngle = angle;
}

float GetRobotAngle(void) {
  return robotAngle;
}