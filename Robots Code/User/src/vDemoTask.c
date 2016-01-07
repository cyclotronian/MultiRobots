#include "vDemoTask.h"
#include "robot.h"
#include "vInfoList.h"
#include "colAvoidance.h"
#include <math.h>

#define LINESPEED 20
#define ANGLESPEED 5
#define SENDTIMES 3

//#define DEBUG




const typeCoordinate posA1 = {0.7581,0.4117};
const typeCoordinate posA2 = {0.6009,0.3984};
const typeCoordinate posA3 = {0.4919,0.3919};
const typeCoordinate posA4 = {0.3568,0.3855};

const typeCoordinate posM  = {1.1220, 0.4303};

const typeCoordinate posC1 = {1.35145,0.25347};
const typeCoordinate posC3 = {1.34659,0.40659};
const typeCoordinate posC4 = {1.26782,0.55151};
const typeCoordinate posC2 = {1.33652,0.70767};

const typeCoordinate posO1 = {0.74572,0.08559};
const typeCoordinate posO3 = {0.73697,0.27856};
const typeCoordinate posO2 = {0.75642,0.52120};
const typeCoordinate posO4 = {0.76721,0.70995};

u8 imready = 0;

float GetPosX();
float GetPosY();

void halt(float time){
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
  vTaskDelay(time*1000);
}
// return which side C is in related to AB
int whichSide (float ax, float ay, float bx, float by, float cx, float cy) {
  float eps = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
  if (eps < -0.02) return -1; // right side
  if (eps > 0.02) return 1; // left side
  return 0; // on the line
}

float getDistance2(float Ax, float Ay, float Bx, float By) {
  return sqrt( (Ax-Bx)*(Ax-Bx) + (Ay-By)*(Ay-By));
}
void broardCastInfo(u8 isReady){
  static volatile rbNode rbInfo;
  rbInfo.nodeID = rid;
  //init the location
  rbInfo.rpos.locationX = GetPosX();
  rbInfo.rpos.locationY = GetPosY();
  //init the north angel
  static u8 txBuf[10] = {0}; //info package
  //send the package
  txBuf[0] = rbInfo.nodeID;	
  memcpy(txBuf+1,(u8 *)(&(rbInfo.rpos.locationX)),1);
  memcpy(txBuf+2,((u8 *)(&(rbInfo.rpos.locationX))+1),1);
  memcpy(txBuf+3,((u8 *)(&(rbInfo.rpos.locationX))+2),1);
  memcpy(txBuf+4,((u8 *)(&(rbInfo.rpos.locationX))+3),1);
  memcpy(txBuf+5,(u8 *)(&(rbInfo.rpos.locationY)),1);
  memcpy(txBuf+6,((u8 *)(&(rbInfo.rpos.locationY))+1),1);
  memcpy(txBuf+7,((u8 *)(&(rbInfo.rpos.locationY))+2),1);
  memcpy(txBuf+8,((u8 *)(&(rbInfo.rpos.locationY))+3),1);
  txBuf[9] = isReady;
  RFTxPacket(RF_BROADCAST_INFO, txBuf, 10);
}

void boardCastInfos(){
  int i = 0;
  for (i = 0; i < SENDTIMES; ++ i) {
    broardCastInfo(1);
  }
}
void rotateTo(float x, float y, float speed) {
  typeCoordinate start = GetCoordinate();
  float lineDir = GetLineDirection(start.x, start.y, x, y);
  float robotDir = ReadMagSensorAngle2North();
  float turnangle = lineDir - robotDir;
  if (turnangle < -180) turnangle += 360;
  if (turnangle > 180) turnangle -= 360;
  ControlRobotRotate(turnangle, speed);
}
void ControlRobotgo2Position(float x, float y, float speed) {
  rotateTo(x,y,ANGLESPEED);
  float speedL = speed;
  float speedR = speed;
  typeCoordinate start = GetCoordinate();
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
  typeCoordinate nowp = GetCoordinate();
  while (1) {
    float dist = getDistance2(nowp.x, nowp.y, x, y);
    if (dist < 0.05) break;
    int side = whichSide(start.x,start.y, x, y, nowp.x, nowp.y);
    if (side == -1) { // right side
      speedL = speed;
      speedR = speed+3;
    } else if (side == 1) { // left side
      speedL = speed+3;
      speedR = speed;
    } else {
      speedL = speedR = speed;
    }
    SetLeftWheelGivenSpeed(speedL);
    SetRightWheelGivenSpeed(speedR);
    vTaskDelay(500);
    nowp = GetCoordinate();
  }
  halt(3);
}
void robot1(){
  imready = 0;
  ControlRobotgo2Position(posA1.x,posA1.y,LINESPEED);
  rotateTo(posM.x,posM.y,ANGLESPEED);
#ifdef DEBUG
  imready = 1;
#else
  while (1) {
    if (isReady(2) && isReady(3) && isReady(4)) {
      imready = 1;
      break;
    }
    halt(0.3);
  }
#endif
  if (imready) {
    boardCastInfos();
    halt(2);
    ControlRobotgo2Position(posM.x,posM.y,LINESPEED);
    ControlRobotgo2Position(posC1.x,posC1.y,LINESPEED);
    rotateTo(posO1.x,posO1.y,ANGLESPEED);
  }
}

void robot2(){
  imready = 0;
  ControlRobotgo2Position(posA2.x,posA2.y,LINESPEED);
  rotateTo(posM.x,posM.y,ANGLESPEED);
#ifdef DEBUG
  imready = 1;
#else
  boardCastInfos();
  while (1) {
    if (isReady(1)) {
      imready = 1;
      break;
    }
    halt(0.3);
  }
#endif
  halt(4.5);
  if (imready) {
    ControlRobotgo2Position(posM.x,posM.y,LINESPEED);
    ControlRobotgo2Position(posC2.x,posC2.y,LINESPEED);
    rotateTo(posO2.x,posO2.y,ANGLESPEED);
  }
}

void robot3(){
  imready = 0;
  ControlRobotgo2Position(posA3.x,posA3.y,LINESPEED);
  rotateTo(posM.x,posM.y,ANGLESPEED);
#ifdef DEBUG
  imready = 1;
#else
  boardCastInfos();
  while (1) {
    if (isReady(1)) {
      imready = 1;
      break;
    }
    halt(0.3);
  }
#endif
  halt(6.5);
  if (imready) {
    ControlRobotgo2Position(posM.x,posM.y,LINESPEED);
    ControlRobotgo2Position(posC3.x,posC3.y,LINESPEED);
    rotateTo(posO3.x,posO3.y,ANGLESPEED);
  }
}

void robot4(){
  imready = 0;
  ControlRobotgo2Position(posA4.x,posA4.y,LINESPEED);
  rotateTo(posM.x,posM.y,ANGLESPEED);
#ifdef DEBUG
  imready = 1;
#else
  boardCastInfos();
  while (1) {
    if (isReady(1)) {
      imready = 1;
      break;
    }
    halt(0.3);
  }
#endif
  halt(8.5);
  if (imready) {
    ControlRobotgo2Position(posM.x,posM.y,LINESPEED);
    ControlRobotgo2Position(posC4.x,posC4.y,LINESPEED);
    rotateTo(posO4.x,posO4.y,ANGLESPEED);
  }
}

void vDemoTask( void *pvParameters ){
  halt(3);
  while (1) {
#ifdef CONFIG_ROBOT1
    robot1();
#elif defined(CONFIG_ROBOT2)
    robot2();
#elif defined(CONFIG_ROBOT3)
    robot3();
#elif defined(CONFIG_ROBOT4)
    robot4();
#endif
    halt(1000);
  }
}
