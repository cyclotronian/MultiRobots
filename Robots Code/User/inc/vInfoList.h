
#ifndef VINFOLIST_H
#define VINFOLIST_H

#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "vTasks.h"
#include "apps.h"
#include "stdbool.h"

#define ROBOTS  2
#define CONFIG_ROBOT3
#define EPS 0.03

//#ifdef CONFIG_ROBOT1
//const static int rid = 1;
//#elif defined(CONFIG_ROBOT2)
//const static int rid = 2;
//#elif defined(CONFIG_ROBOT3)
//const static int rid = 3;
//#elif defined(CONFIG_ROBOT4)
//const static int rid = 4;
//#endif

const static int rid = MY_NODE_ID;

#ifdef CONFIG_ROBOT1
#define MAG_SENSOR_X    (-30.5)
#define MAG_SENSOR_Y    (415.5)
#elif defined(CONFIG_ROBOT2)
#define MAG_SENSOR_X    (93)
#define MAG_SENSOR_Y    (-42)
#elif defined(CONFIG_ROBOT3)
#define MAG_SENSOR_X    (-104)
#define MAG_SENSOR_Y    (643)
#elif defined(CONFIG_ROBOT4)
#define MAG_SENSOR_X    (340.5)
#define MAG_SENSOR_Y    (808.5)
#elif defined(CONFIG_ROBOTNONE)
#define MAG_SENSOR_X    (31)
#define MAG_SENSOR_Y    (687)
#endif

typedef struct robotPos{
  float locationX;        //4bytes
  float locationY;        //4bytes
}robotPos;                //8bytes
  
typedef struct rbNode
{
	u8 nodeID;           //1 byte	
	robotPos rpos;       //8 bytes
        int lvalue;          //4 bytes
	int isReady;         //4 byte
        //u8 isReady1;
}rbNode;

typedef struct InfoNode
{
   struct rbNode rbInfo;
   struct InfoNode *next;
}InfoNode,*pInfoNode;    //24Bytes

//not used
typedef struct InfoDg{
 	u8 nodeID;
	float relDis;
}InfoDg;

  
pInfoNode robotListInit();
void robotListCreatT(InfoNode **L);
pInfoNode robotListInsert(pInfoNode L,int i,rbNode rbInfo);
pInfoNode robotListDelete(pInfoNode L,rbNode rbInfo);
void robotListDestroy(pInfoNode L);
InfoNode getDgNodeFromList(InfoNode *p,robotPos rb,int *flag);
float getRelAngelFromList();
robotPos getRobotPos(InfoNode *rb);

extern float getDistance(robotPos posB,robotPos posC);
extern rbNode recBoardCastInfo(u8 id);


#endif /*end of VINFOLIST_Hs*/