#ifndef __ROBOT_H
#define __ROBOT_H


#include "apps.h"
#include "halDriver.h"

#define M_PI  3.14159265358979323846f
#define DISTANSE_B1_2_B2         0.48
#define DISTANSE_B_2_GROUND      0.48

#define MAX_ROTATE_SPEED         (15)

#define ROBOT_STABLE_ACCEL_X_THR (0.08)
#define ROBOT_STABLE_ACCEL_Y_THR (0.01)
#define ROBOT_STABLE_RETRY_THR   (20)


typedef struct typeCoordinate{
  float x;
  float y;
}typeCoordinate;

extern typeCoordinate GetCoordinate(void);
extern void ControlRobotRotate(float angle, float speed);
extern void RobotPrepare(void);
extern float GetRobotAngle(void);
extern void SetRobotAngle(float angle);
extern float GetLineDirection(float x0, float y0, float x1, float y1);
extern void northRotate(float x, float y, float speed);
extern float ReadMagSensorAngle2North(void);
#endif


