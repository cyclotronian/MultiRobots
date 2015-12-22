#include "multiRobotVM.h"
//#include "robot.h"
//#include "vInfoList.h"
//#include <math.h>

// #define DEBUG_INSTRS
// #define LOG_DEBUG
#define myassert(e) ((e) ? (void)0 : __myassert(__FILE__, __LINE__,#e))

/****************ROBOTIC SUPPORTING FUNCTIONS********************/
#define LINESPEED 20
#define ANGLESPEED 5
#define SENDTIMES 3

u8 imready = 0;

float GetPosX();
float GetPosY();

void halt(int time){
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
  vTaskDelay(time*1000);
}
// return which side C is in related to AB
int whichSide (float ax, float ay, float bx, float by, float cx, float cy) {
  float eps = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
  //halt(1);
  asm("NOP");
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

void broadCastDestInfo(float Lvalue, u8 isReady){
  static volatile rbNode rbInfo;
  rbInfo.nodeID = rid;
  //init the location
  rbInfo.rpos.locationX = GetPosX();
  rbInfo.rpos.locationY = Lvalue;
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
  asm("NOP");
}

void broadCastDestInfos(int x, int y, float Lvalue){
  int i = 0;
  for (i = 0; i < SENDTIMES; ++ i) {
    broadCastDestInfo(Lvalue,0);
  }
}

//void rotateTo(float x, float y, float speed) {
//  typeCoordinate start = GetCoordinate();
//  float lineDir = GetLineDirection(start.x, start.y, x, y);
//  float robotDir = ReadMagSensorAngle2North();
//  float turnangle = lineDir - robotDir;
//  if (turnangle < -180) turnangle += 360;
//  if (turnangle > 180) turnangle -= 360;
//  ControlRobotRotate(turnangle, speed);
//}
//void ControlRobotgo2Position(float x, float y, float speed) {
//  rotateTo(x,y,ANGLESPEED);
//  float speedL, speedR;
//  speedL = speedR = speed;
//  typeCoordinate start = GetCoordinate();
//  SetLeftWheelGivenSpeed(1);
//  SetRightWheelGivenSpeed(1);
//  vTaskDelay(500);
//  SetLeftWheelGivenSpeed(3);
//  SetRightWheelGivenSpeed(3);
//  vTaskDelay(500);
//  SetLeftWheelGivenSpeed(5);
//  SetRightWheelGivenSpeed(5);
//  vTaskDelay(500);
//  SetLeftWheelGivenSpeed(10);
//  SetRightWheelGivenSpeed(10);
//  vTaskDelay(300);
//  SetLeftWheelGivenSpeed(15);
//  SetRightWheelGivenSpeed(15); 
//  vTaskDelay(300);
//  SetLeftWheelGivenSpeed(20);
//  SetRightWheelGivenSpeed(20);
//  typeCoordinate nowp = GetCoordinate();
//  while (1) {
//    nowp = GetCoordinate();
//    float dist = getDistance2(nowp.x, nowp.y, x, y);
//    if (dist < 0.05) break;
//    int side = whichSide(start.x,start.y, x, y, nowp.x, nowp.y);
//    if (side == -1) { // right side
//      speedL = speed;
//      speedR = speed+10;
//    } else if (side == 1) { // left side
//      speedL = speed+10;
//      speedR = speed;
//    } else {
//      speedL = speedR = speed;
//    }
//    SetLeftWheelGivenSpeed(speedL);
//    SetRightWheelGivenSpeed(speedR);
//    vTaskDelay(500);
//    
//    //halt(1);
//    asm("NOP");
//  }
////  SetLeftWheelGivenSpeed(0);
////  SetRightWheelGivenSpeed(-10);
////  vTaskDelay(1000);
//  halt(1);
//  asm("NOP");
//}

//  SetLeftWheelGivenSpeed(20);
//  SetRightWheelGivenSpeed(20);
//  vTaskDelay(500);
//  SetLeftWheelGivenSpeed(15);
//  SetRightWheelGivenSpeed(15);
//  vTaskDelay(500);
//  SetLeftWheelGivenSpeed(10);
//  SetRightWheelGivenSpeed(10);
//  vTaskDelay(500);
//  SetLeftWheelGivenSpeed(5);
//  SetRightWheelGivenSpeed(5);
//  vTaskDelay(300);
//  SetLeftWheelGivenSpeed(1);
//  SetRightWheelGivenSpeed(1); 
//  vTaskDelay(300);
//   SetLeftWheelGivenSpeed(0);
//   SetRightWheelGivenSpeed(0);
//   vTaskDelay(3000);

void rotateTo(float x, float y, float speed,int flag) {
  typeCoordinate start = GetCoordinate();
  float lineDir = GetLineDirection(start.x, start.y, x, y);
  float robotDir = ReadMagSensorAngle2North();
  
  float turnangle = lineDir - robotDir;
  if (turnangle < -180) turnangle += 360;
  if (turnangle > 180) turnangle -= 360;
  if(flag == 0)
  {
	if(FloatAbs(turnangle) <= 15)    //
	  return;
  }
  
  if(flag == 2)  //used for runStraight
  {
	if(FloatAbs(turnangle) <= 25)    //
	  return;
  }
  ControlRobotRotate(turnangle, speed);
}

bool GoalInFront(float x, float y){
	typeCoordinate nowp = GetCoordinate();
	float dist = getDistance2(nowp.x, nowp.y, x,y);
	if( dist < EPS)
		return true;
	else
	    return false;
}


void ControlRobotgo2Position(float x, float y, float speed) {
  typeCoordinate start = GetCoordinate();
  if(GoalInFront(x,y) == true)
	return;
  typeCoordinate nowp;
  int tempTime = 500;
  int  nearTime = 0;
  while (1) {	
	  SetLeftWheelGivenSpeed(speed);
	  SetRightWheelGivenSpeed(speed);
	  vTaskDelay(tempTime - nearTime);
	  if(GoalInFront(x,y) == true){
		break;
          }
          nowp =  GetCoordinate();
          float dist = getDistance2(nowp.x, nowp.y, x, y);
          if(dist >= 0.06)
          {
            rotateTo(x,y,ANGLESPEED,0);	
          }
          else {
            rotateTo(x,y,ANGLESPEED,1);	
            nearTime = 200;
          }
 }  /* end of while */
 halt(1);
 asm("NOP");
} /*end of ControlRobotgo2Position*/

/********************************************************************/

Time getTime(){
  currentDate = xTaskGetTickCount();
  asm("NOP");
  return currentDate;
}

Time myGetTime(){
  // simulator time is in us while the unit in the vm is ms.
  return (Time)(getTime());
}

/* Print the content of the newTuples queue */
void print_newTuples(void)
{

}

byte getNeighborCount() {
  uint8_t count, i;
  for(count = 0, i = 0; i < NUM_PORTS; ++i) {
    if(get_neighbor_ID(i) != VACANT) {
      count++;
    }
  }
  return count;
}

/* Prints the content of the newStartTuples queue */
void print_newStratTuples(void)
{

}

/* Gets ID of neighbor on face 'face' */
inline NodeID get_neighbor_ID(int face){
  NodeID nid = 0;
  return nid;
}

/* Enqueue a edge tuple */
void enqueue_edge(NodeID neighbor){
  if(TYPE_EDGE == -1) return;
  tuple_t tuple = tuple_alloc(TYPE_EDGE);
  SET_TUPLE_FIELD(tuple, 0, &neighbor);
  record_type newRecord;
  newRecord.count=1;
  newRecord.agg_queue = NULL;
  enqueueNewTuple(tuple, newRecord);
}

/* Enqueue a position tuple */
void enqueue_position(meld_int x, meld_int y, meld_int z) {
  // asm("NOP");//("enqueue position\n");
  if (TYPE_POSITION == -1)
    return;
  tuple_t tuple = tuple_alloc(TYPE_POSITION);
  // asm("NOP");//("position: %d\n",x);
  SET_TUPLE_FIELD(tuple, 0, &x);
  SET_TUPLE_FIELD(tuple, 1, &y);
  SET_TUPLE_FIELD(tuple, 2, &z);
  record_type newRecord;
  newRecord.count = 1;
  newRecord.agg_queue = NULL;
  enqueueNewTuple(tuple, newRecord);
}

void enqueue_ready(u8 nodeId){
  tuple_t tuple=NULL;
  tuple = tuple_alloc(TYPE_READY);
  
  SET_TUPLE_FIELD(tuple, 0, &nodeId);
    
  record_type newRecord;
  newRecord.count = 1;
  newRecord.agg_queue = NULL;
  enqueueNewTuple(tuple, newRecord);
}

/* Enqueue a neighbor or vacant tuple */
void enqueue_face(NodeID neighbor, meld_int face, int isNew){
  tuple_t tuple = NULL;

  if (neighbor <= 0) {
    if(TYPE_VACANT == -1) /* no such predicate in the program */
      return;
    tuple = tuple_alloc(TYPE_VACANT);
    SET_TUPLE_FIELD(tuple, 0, &face);
  }
  else {
    if(TYPE_NEIGHBOR == -1) /* no such predicate in the program */
      return;

    tuple = tuple_alloc(TYPE_NEIGHBOR);
    SET_TUPLE_FIELD(tuple, 0, &neighbor);
    SET_TUPLE_FIELD(tuple, 1, &face);
  }
  record_type newRecord;
  newRecord.count = isNew;
  newRecord.agg_queue = NULL;
  enqueueNewTuple(tuple, newRecord);
}
/* Enqueue a neighborCount tuple */
void enqueue_count(meld_int count, int isNew){
  // asm("NOP");//("enqueue count %d\n", count);
  if(TYPE_NEIGHBORCOUNT == -1) /* no such predicate in the program */
    return;

  tuple_t tuple = tuple_alloc(TYPE_NEIGHBORCOUNT);

  SET_TUPLE_FIELD(tuple, 0, &count);

  record_type newRecord;
  newRecord.count = isNew;
  newRecord.agg_queue = NULL;
  enqueueNewTuple(tuple, newRecord);
}

/* Sends a tuple to Block of ID rt, with or without delay: NEED TO BE CHANGED */
  void tuple_send(tuple_t tuple, NodeID rt, meld_int delay, int isNew) {
    asm("NOP");//("Tuple Send is called! %d\n",delay);
    myassert (TUPLE_TYPE(tuple) < NUM_TYPES);
    if (delay > 0) {
      record_type newRecord;
      newRecord.count = isNew;
      newRecord.agg_queue = NULL;
      p_enqueue(delayedTuples, myGetTime() + delay, tuple, rt, newRecord);
      return;
    }

    NodeID target = rt;

    if (target == blockId) {
      record_type newRecord;
      newRecord.count = isNew;
      newRecord.agg_queue = NULL;
      enqueueNewTuple(tuple, newRecord);
    }
    else {
      schedule(getTime(),EVENT_SEND_MESSAGE_TO_BLOCK);
      
    }
  }
  /* Receive a tuple and enqueue it to both receivedTuples and newTuples: NEED TO BE CHANGED */
  void receive_tuple(int isNew, tuple_t tpl, byte face) {
    asm("NOP");//("Tuple received is called!\n");
  }

//#define MAX_STRING_SIZE 200
  /* Prints a tuple */


/* Enqueue a tuple for execution */
void enqueueNewTuple(tuple_t tuple, record_type isNew){
  myassert (TUPLE_TYPE(tuple) < NUM_TYPES);
  if (TYPE_IS_STRATIFIED(TUPLE_TYPE(tuple))) {
    // asm("NOP");//("Before Position!\n");
    p_enqueue(newStratTuples, TYPE_STRATIFICATION_ROUND(TUPLE_TYPE(tuple)), tuple, 0, isNew);
    // if(!queue_is_empty(newTuples)) {
  }
  else {
      queue_enqueue(newTuples, tuple, isNew);
  }
  //tuple_print(tuple, stdout);
 // puts("");
}

/*************** HELPER FUNCTIONS FOR READ PROGRAM ***********************/
int characterCount(const char  *in, char character){
    int i;
    int nCount = 0;
    for(i=0;in[i]!='\0';i++){
      if(in[i]==character){
        nCount++;
      }
    }
    return nCount;
}

int find(const char *in, char character, int start){
    int i;
    // asm("NOP");//("start: %d in: %c%c\n", start,in[0],in[1]);
    int nCount = 0;
    for(i=start;in[i]!='\0';i++){
      nCount++;
      if(in[i]==character){
        break;
      }
    }
    // asm("NOP");//("nCount: %d\n", nCount);
    return nCount;
}

//2 robots 
const char prog[] = "const unsigned char meld_prog[] = {/* NUMBER OF PREDICATES */0xb, /* NUMBER OF RULES */0x5, /* OFFSETS TO PREDICATE DESCRIPTORS */0x22, 0, 0x28, 0, 0x31, 0, 0x39, 0, 0x3f, 0, 0x46, 0, 0x4c, 0, 0x53, 0, 0x5a, 0, 0x60, 0, 0x66, 0, /* OFFSETS TO RULE DESCRIPTORS */0x6c, 0, 0x71, 0, 0x77, 0, 0x7d, 0, 0x83, 0, /* PREDICATE DESCRIPTORS */0x89, 0, 0x4, 0, 0, 0, 0x8a, 0, 0x4, 0, 0x1, 0x3, 0, 0, 0, 0x8b, 0, 0x12, 0, 0x1, 0x2, 0, 0, 0x8c, 0, 0x4, 0, 0x1, 0, 0x8d, 0, 0x22, 0, 0, 0x1, 0x2, 0x8e, 0, 0x4, 0, 0x1, 0, 0x8f, 0, 0x4, 0, 0x1, 0x1, 0x2, 0x90, 0, 0x4, 0, 0x1, 0x1, 0, 0x91, 0, 0x4, 0, 0x1, 0, 0x92, 0, 0x4, 0, 0x1, 0, 0x93, 0, 0x4, 0, 0x1, 0, /* RULE DESCRIPTORS */0x94, 0, 0, 0x1, 0, 0xd4, 0, 0, 0x2, 0x9, 0x1, 0x73, 0x1, 0, 0x2, 0x5, 0xa, 0xd4, 0x1, 0, 0x2, 0x6, 0x7, 0x6d, 0x2, 0, 0x2, 0x1, 0x8, /* PREDICATE BYTECODE *//* Predicate 0: */0, /* Predicate 1: */0, /* Predicate 2: */0, /* Predicate 3: */0, /* Predicate 4: */0, /* Predicate 5: */0, /* Predicate 6: */0, /* Predicate 7: */0, /* Predicate 8: */0, /* Predicate 9: */0, /* Predicate 10: */0, /* RULE BYTECODE *//* Rule 0: */0x10, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x15, 0, 0, 0, 0x3a, 0, 0, 0, 0, 0x11, 0x80, 0, 0x40, 0x7, 0, 0x1e, 0, 0, 0, 0, 0, 0, 0x77, 0, 0x40, 0x5, 0, 0x77, 0, 0x40, 0x9, 0, 0x77, 0, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xf0, 0x1, 0, /* Rule 1: */0x10, 0x1, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x9, 0, 0x1, 0x15, 0, 0, 0, 0x56, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x15, 0, 0, 0, 0x40, 0, 0, 0, 0, 0x22, 0x1, 0x1, 0x2, 0x1f, 0x7c, 0x1, 0, 0, 0x3, 0x3c, 0x2, 0x3, 0x2, 0x60, 0x2, 0x1c, 0, 0, 0, 0x40, 0x2, 0x2, 0x21, 0, 0x1, 0, 0x2, 0x1e, 0x7c, 0x1, 0, 0, 0x1, 0x2, 0x79, 0x2, 0x80, 0x1, 0x7b, 0, 0xf0, 0x1, 0x1, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x9, 0, 0x1, 0x15, 0, 0, 0, 0x43, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x1b, 0, 0, 0, 0x2d, 0, 0, 0, 0x1, 0x1, 0x1, 0x7c, 0x1, 0, 0, 0x40, 0xa, 0x2, 0x77, 0x2, 0x1e, 0x7c, 0x1, 0, 0, 0x1, 0x1, 0x80, 0, 0x7b, 0x1, 0xf0, 0x1, 0x1, 0, /* Rule 2: */0x10, 0x2, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0xa, 0, 0x1, 0x15, 0, 0, 0, 0x5b, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x5, 0x1, 0x1, 0x15, 0, 0, 0, 0x45, 0, 0, 0, 0, 0xe, 0x2a, 0, 0, 0, 0x2, 0, 0, 0, 0, 0, 0, 0, 0, 0x4, 0x2, 0x1, 0x15, 0, 0, 0, 0x24, 0, 0, 0, 0, 0x40, 0x6, 0x3, 0x28, 0, 0x3, 0x22, 0, 0x2, 0x4, 0x8, 0x3, 0x4, 0xf0, 0x1, 0xf, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0, /* Rule 3: */0x10, 0x3, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x6, 0, 0x1, 0x15, 0, 0, 0, 0x57, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x7, 0x1, 0x1, 0x15, 0, 0, 0, 0x41, 0, 0, 0, 0, 0x22, 0, 0x1, 0x2, 0x1f, 0x1, 0, 0, 0, 0x3, 0x3e, 0x2, 0x3, 0x2, 0x60, 0x2, 0x1d, 0, 0, 0, 0x22, 0, 0x1, 0x2, 0x1f, 0x1, 0, 0, 0, 0x3, 0x3d, 0x2, 0x3, 0x2, 0x26, 0x2, 0, 0x1, 0x80, 0, 0x7b, 0x1, 0xf0, 0x1, 0x1, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x6, 0, 0x1, 0x15, 0, 0, 0, 0x3c, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x7, 0x1, 0x1, 0x1b, 0, 0, 0, 0x26, 0, 0, 0, 0x1, 0, 0x1, 0x1, 0, 0, 0, 0x40, 0x8, 0x2, 0x77, 0x2, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0, /* Rule 4: */0x10, 0x4, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x8, 0, 0x1, 0x15, 0, 0, 0, 0x4d, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x15, 0, 0, 0, 0x37, 0, 0, 0, 0, 0x40, 0x2, 0x2, 0x22, 0, 0x1, 0x3, 0x1f, 0xdc, 0x5, 0, 0, 0x4, 0x3d, 0x3, 0x4, 0x3, 0x26, 0x3, 0, 0x2, 0x21, 0x1, 0x1, 0x1, 0x2, 0x79, 0x2, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0, };char *tuple_names[] = {\"_init\", \"position\", \"moveto\", \"broadcast\", \"edge\", \"unbroadcasted\", \"ready\", \"readycount\", \"movealong\", \"offline\", \"online\", };char *rule_names[] = {\"_init -o node-axioms.\", \" -o (offline(), position(X, Y, Z), Y != 380 -o offline(), 			!moveTo(X, 380)), OR (offline(), position(X, Y, Z), Y = 380 -o online(), 			position(X, 380, Z)).\", \"online(), unbroadcasted() -o {B | !edge(B) | 			ready(host-id)@B}.\", \" -o (ready(B), readycount(N), N < 1 -o readycount(N + 1)), OR (ready(B), readycount(N), N = 1 -o movealong()).\", \"movealong(), position(X, Y, Z) -o !moveTo(X + 1500, Y).\", };#include \"extern_functions.bbh\"Register (*extern_functs[])() = {};int extern_functs_args[] = {};";
/********** FUNCTION TO READ THE PROGRAM ***********************************/
const char progString[] = "/* NUMBER OF PREDICATES */0xb, /* NUMBER OF RULES */0x5, /* OFFSETS TO PREDICATE DESCRIPTORS */0x22, 0, 0x28, 0, 0x31, 0, 0x39, 0, 0x3f, 0, 0x46, 0, 0x4c, 0, 0x53, 0, 0x5a, 0, 0x60, 0, 0x66, 0, /* OFFSETS TO RULE DESCRIPTORS */0x6c, 0, 0x71, 0, 0x77, 0, 0x7d, 0, 0x83, 0, /* PREDICATE DESCRIPTORS */0x89, 0, 0x4, 0, 0, 0, 0x8a, 0, 0x4, 0, 0x1, 0x3, 0, 0, 0, 0x8b, 0, 0x12, 0, 0x1, 0x2, 0, 0, 0x8c, 0, 0x4, 0, 0x1, 0, 0x8d, 0, 0x22, 0, 0, 0x1, 0x2, 0x8e, 0, 0x4, 0, 0x1, 0, 0x8f, 0, 0x4, 0, 0x1, 0x1, 0x2, 0x90, 0, 0x4, 0, 0x1, 0x1, 0, 0x91, 0, 0x4, 0, 0x1, 0, 0x92, 0, 0x4, 0, 0x1, 0, 0x93, 0, 0x4, 0, 0x1, 0, /* RULE DESCRIPTORS */0x94, 0, 0, 0x1, 0, 0xd4, 0, 0, 0x2, 0x9, 0x1, 0x73, 0x1, 0, 0x2, 0x5, 0xa, 0xd4, 0x1, 0, 0x2, 0x6, 0x7, 0x6d, 0x2, 0, 0x2, 0x1, 0x8, /* PREDICATE BYTECODE *//* Predicate 0: */0, /* Predicate 1: */0, /* Predicate 2: */0, /* Predicate 3: */0, /* Predicate 4: */0, /* Predicate 5: */0, /* Predicate 6: */0, /* Predicate 7: */0, /* Predicate 8: */0, /* Predicate 9: */0, /* Predicate 10: */0, /* RULE BYTECODE *//* Rule 0: */0x10, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x15, 0, 0, 0, 0x3a, 0, 0, 0, 0, 0x11, 0x80, 0, 0x40, 0x7, 0, 0x1e, 0, 0, 0, 0, 0, 0, 0x77, 0, 0x40, 0x5, 0, 0x77, 0, 0x40, 0x9, 0, 0x77, 0, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xf0, 0x1, 0, /* Rule 1: */0x10, 0x1, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x9, 0, 0x1, 0x15, 0, 0, 0, 0x56, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x15, 0, 0, 0, 0x40, 0, 0, 0, 0, 0x22, 0x1, 0x1, 0x2, 0x1f, 0x7c, 0x1, 0, 0, 0x3, 0x3c, 0x2, 0x3, 0x2, 0x60, 0x2, 0x1c, 0, 0, 0, 0x40, 0x2, 0x2, 0x21, 0, 0x1, 0, 0x2, 0x1e, 0x7c, 0x1, 0, 0, 0x1, 0x2, 0x79, 0x2, 0x80, 0x1, 0x7b, 0, 0xf0, 0x1, 0x1, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x9, 0, 0x1, 0x15, 0, 0, 0, 0x43, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x1b, 0, 0, 0, 0x2d, 0, 0, 0, 0x1, 0x1, 0x1, 0x7c, 0x1, 0, 0, 0x40, 0xa, 0x2, 0x77, 0x2, 0x1e, 0x7c, 0x1, 0, 0, 0x1, 0x1, 0x80, 0, 0x7b, 0x1, 0xf0, 0x1, 0x1, 0, /* Rule 2: */0x10, 0x2, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0xa, 0, 0x1, 0x15, 0, 0, 0, 0x5b, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x5, 0x1, 0x1, 0x15, 0, 0, 0, 0x45, 0, 0, 0, 0, 0xe, 0x2a, 0, 0, 0, 0x2, 0, 0, 0, 0, 0, 0, 0, 0, 0x4, 0x2, 0x1, 0x15, 0, 0, 0, 0x24, 0, 0, 0, 0, 0x40, 0x6, 0x3, 0x28, 0, 0x3, 0x22, 0, 0x2, 0x4, 0x8, 0x3, 0x4, 0xf0, 0x1, 0xf, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0, /* Rule 3: */0x10, 0x3, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x6, 0, 0x1, 0x15, 0, 0, 0, 0x57, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x7, 0x1, 0x1, 0x15, 0, 0, 0, 0x41, 0, 0, 0, 0, 0x22, 0, 0x1, 0x2, 0x1f, 0x1, 0, 0, 0, 0x3, 0x3e, 0x2, 0x3, 0x2, 0x60, 0x2, 0x1d, 0, 0, 0, 0x22, 0, 0x1, 0x2, 0x1f, 0x1, 0, 0, 0, 0x3, 0x3d, 0x2, 0x3, 0x2, 0x26, 0x2, 0, 0x1, 0x80, 0, 0x7b, 0x1, 0xf0, 0x1, 0x1, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x6, 0, 0x1, 0x15, 0, 0, 0, 0x3c, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x7, 0x1, 0x1, 0x1b, 0, 0, 0, 0x26, 0, 0, 0, 0x1, 0, 0x1, 0x1, 0, 0, 0, 0x40, 0x8, 0x2, 0x77, 0x2, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0, /* Rule 4: */0x10, 0x4, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x8, 0, 0x1, 0x15, 0, 0, 0, 0x4d, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x15, 0, 0, 0, 0x37, 0, 0, 0, 0, 0x40, 0x2, 0x2, 0x22, 0, 0x1, 0x3, 0x1f, 0xdc, 0x5, 0, 0, 0x4, 0x3d, 0x3, 0x4, 0x3, 0x26, 0x3, 0, 0x2, 0x21, 0x1, 0x1, 0x1, 0x2, 0x79, 0x2, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0,";
const char tupleString[] = "\"_init\", \"position\", \"moveto\", \"broadcast\", \"edge\", \"unbroadcasted\", \"ready\", \"readycount\", \"movealong\", \"offline\", \"online\",";

//4 robots
//const char prog[] = "const unsigned char meld_prog[] = {/* NUMBER OF PREDICATES */0xb, /* NUMBER OF RULES */0x5, /* OFFSETS TO PREDICATE DESCRIPTORS */0x22, 0, 0x28, 0, 0x31, 0, 0x39, 0, 0x3f, 0, 0x46, 0, 0x4c, 0, 0x53, 0, 0x5a, 0, 0x60, 0, 0x66, 0, /* OFFSETS TO RULE DESCRIPTORS */0x6c, 0, 0x71, 0, 0x77, 0, 0x7d, 0, 0x83, 0, /* PREDICATE DESCRIPTORS */0x89, 0, 0x4, 0, 0, 0, 0x8a, 0, 0x4, 0, 0x1, 0x3, 0, 0, 0, 0x8b, 0, 0x12, 0, 0x1, 0x2, 0, 0, 0x8c, 0, 0x4, 0, 0x1, 0, 0x8d, 0, 0x22, 0, 0, 0x1, 0x2, 0x8e, 0, 0x4, 0, 0x1, 0, 0x8f, 0, 0x4, 0, 0x1, 0x1, 0x2, 0x90, 0, 0x4, 0, 0x1, 0x1, 0, 0x91, 0, 0x4, 0, 0x1, 0, 0x92, 0, 0x4, 0, 0x1, 0, 0x93, 0, 0x4, 0, 0x1, 0, /* RULE DESCRIPTORS */0x94, 0, 0, 0x1, 0, 0xd4, 0, 0, 0x2, 0x9, 0x1, 0x73, 0x1, 0, 0x2, 0x5, 0xa, 0xd4, 0x1, 0, 0x2, 0x6, 0x7, 0x6d, 0x2, 0, 0x2, 0x1, 0x8, /* PREDICATE BYTECODE *//* Predicate 0: */0, /* Predicate 1: */0, /* Predicate 2: */0, /* Predicate 3: */0, /* Predicate 4: */0, /* Predicate 5: */0, /* Predicate 6: */0, /* Predicate 7: */0, /* Predicate 8: */0, /* Predicate 9: */0, /* Predicate 10: */0, /* RULE BYTECODE *//* Rule 0: */0x10, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x15, 0, 0, 0, 0x3a, 0, 0, 0, 0, 0x11, 0x80, 0, 0x40, 0x7, 0, 0x1e, 0, 0, 0, 0, 0, 0, 0x77, 0, 0x40, 0x5, 0, 0x77, 0, 0x40, 0x9, 0, 0x77, 0, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xf0, 0x1, 0, /* Rule 1: */0x10, 0x1, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x9, 0, 0x1, 0x15, 0, 0, 0, 0x56, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x15, 0, 0, 0, 0x40, 0, 0, 0, 0, 0x22, 0x1, 0x1, 0x2, 0x1f, 0x7c, 0x1, 0, 0, 0x3, 0x3c, 0x2, 0x3, 0x2, 0x60, 0x2, 0x1c, 0, 0, 0, 0x40, 0x2, 0x2, 0x21, 0, 0x1, 0, 0x2, 0x1e, 0x7c, 0x1, 0, 0, 0x1, 0x2, 0x79, 0x2, 0x80, 0x1, 0x7b, 0, 0xf0, 0x1, 0x1, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x9, 0, 0x1, 0x15, 0, 0, 0, 0x43, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x1b, 0, 0, 0, 0x2d, 0, 0, 0, 0x1, 0x1, 0x1, 0x7c, 0x1, 0, 0, 0x40, 0xa, 0x2, 0x77, 0x2, 0x1e, 0x7c, 0x1, 0, 0, 0x1, 0x1, 0x80, 0, 0x7b, 0x1, 0xf0, 0x1, 0x1, 0, /* Rule 2: */0x10, 0x2, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0xa, 0, 0x1, 0x15, 0, 0, 0, 0x5b, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x5, 0x1, 0x1, 0x15, 0, 0, 0, 0x45, 0, 0, 0, 0, 0xe, 0x2a, 0, 0, 0, 0x2, 0, 0, 0, 0, 0, 0, 0, 0, 0x4, 0x2, 0x1, 0x15, 0, 0, 0, 0x24, 0, 0, 0, 0, 0x40, 0x6, 0x3, 0x28, 0, 0x3, 0x22, 0, 0x2, 0x4, 0x8, 0x3, 0x4, 0xf0, 0x1, 0xf, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0, /* Rule 3: */0x10, 0x3, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x6, 0, 0x1, 0x15, 0, 0, 0, 0x57, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x7, 0x1, 0x1, 0x15, 0, 0, 0, 0x41, 0, 0, 0, 0, 0x22, 0, 0x1, 0x2, 0x1f, 0x3, 0, 0, 0, 0x3, 0x3e, 0x2, 0x3, 0x2, 0x60, 0x2, 0x1d, 0, 0, 0, 0x22, 0, 0x1, 0x2, 0x1f, 0x1, 0, 0, 0, 0x3, 0x3d, 0x2, 0x3, 0x2, 0x26, 0x2, 0, 0x1, 0x80, 0, 0x7b, 0x1, 0xf0, 0x1, 0x1, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x6, 0, 0x1, 0x15, 0, 0, 0, 0x3c, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x7, 0x1, 0x1, 0x1b, 0, 0, 0, 0x26, 0, 0, 0, 0x1, 0, 0x1, 0x3, 0, 0, 0, 0x40, 0x8, 0x2, 0x77, 0x2, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0, /* Rule 4: */0x10, 0x4, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x8, 0, 0x1, 0x15, 0, 0, 0, 0x4d, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x15, 0, 0, 0, 0x37, 0, 0, 0, 0, 0x40, 0x2, 0x2, 0x22, 0, 0x1, 0x3, 0x1f, 0xdc, 0x5, 0, 0, 0x4, 0x3d, 0x3, 0x4, 0x3, 0x26, 0x3, 0, 0x2, 0x21, 0x1, 0x1, 0x1, 0x2, 0x79, 0x2, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0, };char *tuple_names[] = {\"_init\", \"position\", \"moveto\", \"broadcast\", \"edge\", \"unbroadcasted\", \"ready\", \"readycount\", \"movealong\", \"offline\", \"online\", };char *rule_names[] = {\"_init -o node-axioms.\", \" -o (offline(), position(X, Y, Z), Y != 380 -o offline(), 			!moveTo(X, 380)), OR (offline(), position(X, Y, Z), Y = 380 -o online(), 			position(X, 380, Z)).\", \"online(), unbroadcasted() -o {B | !edge(B) | 			ready(host-id)@B}.\", \" -o (ready(B), readycount(N), N < 3 -o readycount(N + 1)), OR (ready(B), readycount(N), N = 3 -o movealong()).\", \"movealong(), position(X, Y, Z) -o !moveTo(X + 1500, Y).\", };#include \"extern_functions.bbh\"Register (*extern_functs[])() = {};int extern_functs_args[] = {};";
///********** FUNCTION TO READ THE PROGRAM ***********************************/
//const char progString[] = "/* NUMBER OF PREDICATES */0xb, /* NUMBER OF RULES */0x5, /* OFFSETS TO PREDICATE DESCRIPTORS */0x22, 0, 0x28, 0, 0x31, 0, 0x39, 0, 0x3f, 0, 0x46, 0, 0x4c, 0, 0x53, 0, 0x5a, 0, 0x60, 0, 0x66, 0, /* OFFSETS TO RULE DESCRIPTORS */0x6c, 0, 0x71, 0, 0x77, 0, 0x7d, 0, 0x83, 0, /* PREDICATE DESCRIPTORS */0x89, 0, 0x4, 0, 0, 0, 0x8a, 0, 0x4, 0, 0x1, 0x3, 0, 0, 0, 0x8b, 0, 0x12, 0, 0x1, 0x2, 0, 0, 0x8c, 0, 0x4, 0, 0x1, 0, 0x8d, 0, 0x22, 0, 0, 0x1, 0x2, 0x8e, 0, 0x4, 0, 0x1, 0, 0x8f, 0, 0x4, 0, 0x1, 0x1, 0x2, 0x90, 0, 0x4, 0, 0x1, 0x1, 0, 0x91, 0, 0x4, 0, 0x1, 0, 0x92, 0, 0x4, 0, 0x1, 0, 0x93, 0, 0x4, 0, 0x1, 0, /* RULE DESCRIPTORS */0x94, 0, 0, 0x1, 0, 0xd4, 0, 0, 0x2, 0x9, 0x1, 0x73, 0x1, 0, 0x2, 0x5, 0xa, 0xd4, 0x1, 0, 0x2, 0x6, 0x7, 0x6d, 0x2, 0, 0x2, 0x1, 0x8, /* PREDICATE BYTECODE *//* Predicate 0: */0, /* Predicate 1: */0, /* Predicate 2: */0, /* Predicate 3: */0, /* Predicate 4: */0, /* Predicate 5: */0, /* Predicate 6: */0, /* Predicate 7: */0, /* Predicate 8: */0, /* Predicate 9: */0, /* Predicate 10: */0, /* RULE BYTECODE *//* Rule 0: */0x10, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x15, 0, 0, 0, 0x3a, 0, 0, 0, 0, 0x11, 0x80, 0, 0x40, 0x7, 0, 0x1e, 0, 0, 0, 0, 0, 0, 0x77, 0, 0x40, 0x5, 0, 0x77, 0, 0x40, 0x9, 0, 0x77, 0, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xf0, 0x1, 0, /* Rule 1: */0x10, 0x1, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x9, 0, 0x1, 0x15, 0, 0, 0, 0x56, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x15, 0, 0, 0, 0x40, 0, 0, 0, 0, 0x22, 0x1, 0x1, 0x2, 0x1f, 0x7c, 0x1, 0, 0, 0x3, 0x3c, 0x2, 0x3, 0x2, 0x60, 0x2, 0x1c, 0, 0, 0, 0x40, 0x2, 0x2, 0x21, 0, 0x1, 0, 0x2, 0x1e, 0x7c, 0x1, 0, 0, 0x1, 0x2, 0x79, 0x2, 0x80, 0x1, 0x7b, 0, 0xf0, 0x1, 0x1, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x9, 0, 0x1, 0x15, 0, 0, 0, 0x43, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x1b, 0, 0, 0, 0x2d, 0, 0, 0, 0x1, 0x1, 0x1, 0x7c, 0x1, 0, 0, 0x40, 0xa, 0x2, 0x77, 0x2, 0x1e, 0x7c, 0x1, 0, 0, 0x1, 0x1, 0x80, 0, 0x7b, 0x1, 0xf0, 0x1, 0x1, 0, /* Rule 2: */0x10, 0x2, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0xa, 0, 0x1, 0x15, 0, 0, 0, 0x5b, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x5, 0x1, 0x1, 0x15, 0, 0, 0, 0x45, 0, 0, 0, 0, 0xe, 0x2a, 0, 0, 0, 0x2, 0, 0, 0, 0, 0, 0, 0, 0, 0x4, 0x2, 0x1, 0x15, 0, 0, 0, 0x24, 0, 0, 0, 0, 0x40, 0x6, 0x3, 0x28, 0, 0x3, 0x22, 0, 0x2, 0x4, 0x8, 0x3, 0x4, 0xf0, 0x1, 0xf, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0, /* Rule 3: */0x10, 0x3, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x6, 0, 0x1, 0x15, 0, 0, 0, 0x57, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x7, 0x1, 0x1, 0x15, 0, 0, 0, 0x41, 0, 0, 0, 0, 0x22, 0, 0x1, 0x2, 0x1f, 0x3, 0, 0, 0, 0x3, 0x3e, 0x2, 0x3, 0x2, 0x60, 0x2, 0x1d, 0, 0, 0, 0x22, 0, 0x1, 0x2, 0x1f, 0x1, 0, 0, 0, 0x3, 0x3d, 0x2, 0x3, 0x2, 0x26, 0x2, 0, 0x1, 0x80, 0, 0x7b, 0x1, 0xf0, 0x1, 0x1, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x6, 0, 0x1, 0x15, 0, 0, 0, 0x3c, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x7, 0x1, 0x1, 0x1b, 0, 0, 0, 0x26, 0, 0, 0, 0x1, 0, 0x1, 0x3, 0, 0, 0, 0x40, 0x8, 0x2, 0x77, 0x2, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0, /* Rule 4: */0x10, 0x4, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x8, 0, 0x1, 0x15, 0, 0, 0, 0x4d, 0, 0, 0, 0, 0x5, 0, 0, 0, 0, 0, 0, 0, 0, 0x1, 0x1, 0x1, 0x15, 0, 0, 0, 0x37, 0, 0, 0, 0, 0x40, 0x2, 0x2, 0x22, 0, 0x1, 0x3, 0x1f, 0xdc, 0x5, 0, 0, 0x4, 0x3d, 0x3, 0x4, 0x3, 0x26, 0x3, 0, 0x2, 0x21, 0x1, 0x1, 0x1, 0x2, 0x79, 0x2, 0x80, 0x1, 0x80, 0, 0xf0, 0x1, 0x1, 0,";
//const char tupleString[] = "\"_init\", \"position\", \"moveto\", \"broadcast\", \"edge\", \"unbroadcasted\", \"ready\", \"readycount\", \"movealong\", \"offline\", \"online\",";


void readProgram(){

    //Separate the different variable value in several string
    //static int pos;

    //Reading meld_prog
    int byteCount = characterCount(progString, ',');
    unsigned char* outProg = (unsigned char*)malloc((byteCount + 1)*sizeof(unsigned char));
    int movingCursor = 0;
    int valByte = 0;
    int leftCursor = 0;
    int rightCursor = 0;
    int x = (int)'0';
    int multi = 1;
    int i=0;
    for(i = 0; i <= byteCount; i++){
      leftCursor = rightCursor;
      rightCursor = find(progString, ',', leftCursor + 1) + leftCursor;
      movingCursor = rightCursor - 1;
      //read the value between the ','
      for(; movingCursor >= leftCursor; movingCursor--){
        x = (int)'0';
        //Test if character is not part of a byte
        while(x <= ((int)'9')){ //Test for 0-9
          if((int)(progString[movingCursor]) == x ) break;
          x++;
        }
        if(x <= (int)'9'){
          valByte += ((((int)(progString[movingCursor])) - (int)'0')* multi);
          multi = multi * 16;
          continue;
        }
        x = (int)'a';
        while(x <= (int)'f'){ //Test for a-f
          if((int)(progString[movingCursor]) == x ) break;
          x++;
        }
        if(x <= (int)'f'){
          valByte += ((((int)(progString[movingCursor]) + 10) - (int)'a')* multi);
          multi = multi * 16;
          continue;
        }
        if((int)'0' < x && x > (int)'9' && (int)'a' < x && x > (int)'f') break; //if value is not hexadecimal
      }
      outProg[i] = (unsigned char)valByte;
      valByte = 0;
      multi = 1;
    }
    meld_prog = outProg;
    //Reading tuple_names
    int countingTuple = characterCount(tupleString, ',');
    char** outTuple = (char**)malloc((countingTuple)*sizeof(char*));

    i = 0;
    int st = find(tupleString, '"', 0)-1, ed, j;
    while (i < countingTuple) {
	ed = find(tupleString, '"', st+1) + st;
	outTuple[i] = (char*) malloc(sizeof(char) * (ed-st));
	for (j = st+1; j < ed; ++ j) {
		outTuple[i][j-st-1] = tupleString[j];
	}
	outTuple[i][ed-st-1] = 0;
	st = find(tupleString, '"', ed+1) + ed;
	i ++;
    }

    tuple_names = outTuple;
}
/* ************* END ***************** */
#define SCHEDULER1
static rbNode broadcastinfo[ROBOTS];

int detectCollision(int x, int y){
  for(int id=1; id<=ROBOTS; id++){
    if(id != MY_NODE_ID){
      if( abs(x - broadcastinfo[id].rpos.locationX) < 100){
        return id;
      }
    }
  }
  return 0;
}

void avoidcollision1(int collisionid, int x, int y, int lvalue){
  if(collisionid == 0 || broadcastinfo[collisionid].rpos.locationY > lvalue){
     targetPos[0]=x;
     targetPos[1]=y;
  }
  else{
     targetPos[0]=x-200;
     targetPos[1]=y;
  }
}

void avoidcollision2(int collisionid, int x, int y, int lvalue){
//  static float id1,id2;
//  id1 = broadcastinfo[collisionid].rpos.locationY;
//  id2 = lvalue;
//  asm("NOP");
//  if(collisionid == 1 || broadcastinfo[collisionid].rpos.locationY < lvalue){
//     targetPos[0]=x;
//     targetPos[1]=y;
//  }
//  else{
     targetPos[0]=x-200;
     targetPos[1]=y;
//  }
}

void avoidcollision3(int x, int y){
     targetPos[0]=x-170;
     targetPos[1]=y;
}
     
  void RealTimeScheduler(int x, int y, int lvalue){
    for(int i=1; i<=ROBOTS; i++){
      if(i!=MY_NODE_ID)
        broadcastinfo[i] = recBoardCastInfo(i);
    }
    //avoidcollision1(detectCollision(x, y), x, y, lvalue);
    //avoidcollision2(detectCollision(x, y), x, y, lvalue);
    avoidcollision3(x, y);
  }

  void setPosition(int x, int y, int z) {
    // BaseSimulator::getScheduler()->schedule(new SetPositionEvent(BaseSimulator::getScheduler()->now(), host , x, y, z));
    //schedule setPosition
    // ControlRobotgo2Position2((x+100)/100.0,(y+100)/100.0,SPEED);
    schedule(getTime(),EVENT_SET_POSITION);
    asm("NOP");//("Scheduling setPosition %d %d %d\n",x,y,z);
  }
  void moveTo(int x, int y, int z) {
//    typeCoordinate nowp = GetCoordinate();
    halt(5);
//    broadCastDestInfos(x,y,abs(nowp.y-y));
//    RealTimeScheduler(x,y,abs(nowp.y-y));
    targetPos[0]=x;
    targetPos[1]=y;
    targetPos[2]=z;
    asm("NOP");//("Scheduling moveTo\n");
    schedule(getTime()+500000,EVENT_MOVE_TO);
    // BaseSimulator::getScheduler()->schedule(new MoveToEvent(BaseSimulator::getScheduler()->now()+500000, host, x, y, z));
  }

int getNextRandomNumber() {
  static int x = 0;
  x = x + 500;
  //do { 
  //} while (x <= 0);
  // asm("NOP");//("RANDOM NO: %d\n", x);
  return x;
}

  /* Iterate through the database to find a match with tuple read from byte code
   * If there are matches, process the inside of the ITER with all matches sequentially.
   */
  int execute_iter (const unsigned char *pc, Register *regis, int isNew, int isLinear) {
    const unsigned char *inner_jump = pc + ITER_INNER_JUMP(pc);
    const tuple_type type = ITER_TYPE(pc);
    int i, k, length;
    void **list;

    /* Reg in which match will be stored during execution*/
    byte reg_store_index = FETCH(pc+10);

    /* produce a random ordering for all tuples of the appropriate type */
    tuple_entry *entry = TUPLES[type].head;

    length = queue_length(&TUPLES[ITER_TYPE(pc)]);
    list = (void**)malloc(sizeof(tuple_t) * length);

    for (i = 0; i < length; i++) {
      int j = getNextRandomNumber() % (i+1);

      list[i] = list[j];
      list[j] = entry->tuple;

      entry = entry->next;
    }

#ifdef DEBUG_INSTRS
    asm("NOP");//("--%d--\t ITER %s len=%d TO regis %d\n",
        getBlockId(), tuple_names[type], length, reg_store_index);
#endif

    if(length == 0) {
      free(list);
      /* no need to execute any further code, just jump! */
      return RET_NO_RET;
    }

    /* iterate over all tuples of the appropriate type */
    void *next_tuple;

    for (i = 0; i < length; i++) {
      next_tuple = list[i];

      unsigned char matched = 1;
      unsigned char num_args = ITER_NUM_ARGS(pc);
      const unsigned char *tmppc = pc + PERS_ITER_BASE;

      /* check to see if it matches */
      for (k = 0; k < num_args; ++k) {
        const unsigned char fieldnum = ITER_MATCH_FIELD(tmppc);
        const unsigned char fieldtype = TYPE_ARG_TYPE(type, fieldnum);
        const unsigned char type_size = TYPE_ARG_SIZE(type, fieldnum);
        const unsigned char value_type = ITER_MATCH_VAL(tmppc);

        Register *field = GET_TUPLE_FIELD(next_tuple, fieldnum);
        Register *val;

        if (val_is_int (value_type)) {
          tmppc += 2;
          val = (Register *)eval_int(&tmppc);
        } else if (val_is_float (value_type)) {
          tmppc += 2;
          val = (Register *)eval_float(&tmppc);
        } else if (val_is_field (value_type)) {
          tmppc += 2;
          byte reg_index = FETCH(tmppc+1);
          tuple_t tpl = (tuple_t)regis[reg_index];
          val = (Register *)eval_field(tpl, &tmppc);
        }  else {
          /* Don't know what to do */
          //fasm("NOP");// (stderr, "Type %d not supported yet - don't know what to do.\n", fieldtype);
          myassert (0);
          exit (2);
        }

        matched = matched && (memcmp(field, val, type_size) == 0);
      }

#ifdef DEBUG_INSTRS
      asm("NOP");//("--%d--\t MATCHED: %d | length: %d\n", getBlockId(),
          matched, length);
#endif

      if (matched) {
        /* We've got a match! */
        moveTupleToReg (reg_store_index, next_tuple, regis);
        /* Process it - And if we encounter a return instruction, return
         * Otherwise, look for another match.
         */
        
        // asm("NOP");//("From execute_iter\n");
        int ret = process_bytecode(next_tuple, inner_jump,
           isNew, isLinear || TYPE_IS_LINEAR(TUPLE_TYPE(next_tuple)), regis, PROCESS_ITER);
        if(ret == RET_LINEAR) {
          free(list);
          return ret;
        }
        if(isLinear && ret == RET_DERIVED) {
          free(list);
          return ret;
        }
        if(ret == RET_RET) {
          free(list);
          return ret;
        }
      }
    }

    free(list);

    /* process next instructions */
    return RET_NO_RET;
  }

inline void execute_run_action0 (tuple_t action_tuple, tuple_type type, int isNew) {
  if (type == TYPE_SETPOSITION) {
    if (isNew > 0) {
#ifdef DEBUG_INSTRS
      asm("NOP");// ("--%d--\t RUN ACTION: %s(currentNode, %d)\n",
          getBlockId(), tuple_names[type],
          MELD_INT(GET_TUPLE_FIELD(action_tuple, 0)));
#endif

      /* Don't call it directly to avoid having to import led.bbh */
      setPosition(MELD_INT(GET_TUPLE_FIELD(action_tuple, 0)),
          MELD_INT(GET_TUPLE_FIELD(action_tuple, 1)),
          0);
    }
    FREE_TUPLE(action_tuple);
  } else if (type == TYPE_MOVETO) {
    if (isNew > 0) {
      moveTo(MELD_INT(GET_TUPLE_FIELD(action_tuple, 0)),
          MELD_INT(GET_TUPLE_FIELD(action_tuple, 1)),
          0);
    }
    FREE_TUPLE(action_tuple);
  }
}

void tuple_do_handle(tuple_type type, tuple_t tuple, int isNew, Register *regis) {
  // asm("NOP");//("REG VALUE: %lu %lu\n", regis[0],regis[1]);
  if(type == TYPE_TERMINATE) {
    FREE_TUPLE(tuple);
    TERMINATE_CURRENT();
    return;
  }

#ifdef DEBUG_INSTRS
  if (isNew == 1) {
    fasm("NOP");//(stderr, "\x1b[1;32m--%d--\tExecuting tuple ", getBlockId());
    //tuple_print (tuple, stderr);
    fasm("NOP");//(stderr, "\x1b[0m\n");
  } else if (isNew == -1) {
    fasm("NOP");//(stderr, "\x1b[1;31m--%d--\tDeleting tuple ", getBlockId());
    //tuple_print (tuple, stderr);
    fasm("NOP");//(stderr, "\x1b[0m\n");
  }
#endif

  if (TYPE_IS_ACTION(type)) {
    if(isNew > 0)
      execute_run_action0(tuple, type, isNew);
    else
      FREE_TUPLE(tuple);
    return;
  }

  if (!TYPE_IS_AGG(type) || TYPE_IS_LINEAR(type)) {
    tuple_queue *queue = &TUPLES[type];
    tuple_entry** current;
    tuple_entry* cur;

    for (current = &queue->head; *current != NULL; current = &(*current)->next) {
      cur = *current;
      if (memcmp(cur->tuple, tuple, TYPE_SIZE(type)) == 0) {
        cur->records.count += isNew;

        if (cur->records.count <= 0) {
          /* Remove fact from database */
          if (!TYPE_IS_LINEAR(type))
            // asm("NOP");//("From tuple_do_handle 1\n");
            process_bytecode(tuple, TYPE_START(TUPLE_TYPE(tuple)), isNew,
                NOT_LINEAR, regis, PROCESS_TUPLE);
#ifdef DEBUG_INSTRS
          fasm("NOP");//(stdout, "\x1b[1;32m--%d--\tDelete Iter success for  %s\x1b[0m\n", getBlockId(), tuple_names[type]);
#endif
          FREE_TUPLE(queue_dequeue_pos(queue, current));
          /* Also free retraction fact */
          FREE_TUPLE(tuple);

          return;
        }

        if(isNew > 0 && !TYPE_IS_LINEAR(type)) {
          /* tuple found, no need to rederive */
          FREE_TUPLE(tuple);
          return;
        }
      }
    }

    // if deleting, return
    if (isNew <= 0) {
#ifdef DEBUG_INSTRS
      fasm("NOP");//(stdout, "\x1b[1;31m--%d--\tDelete Iter failure for %s\x1b[0m\n", getBlockId(), tuple_names[type]);
#endif
      FREE_TUPLE(tuple);
      return;
    }
    record_type newRecord;
    newRecord.count = isNew;
    newRecord.agg_queue = NULL;
    queue_enqueue(queue, tuple, newRecord);
    // asm("NOP");//("From tuple_do_handle 2\n");
    process_bytecode(tuple, TYPE_START(TUPLE_TYPE(tuple)), isNew, TYPE_IS_LINEAR(TUPLE_TYPE(tuple)), regis, PROCESS_TUPLE);
    return;
  }

  unsigned char type_aggregate = TYPE_AGGREGATE(type);
  unsigned char field_aggregate = AGG_FIELD(type_aggregate);

  tuple_entry **current;
  tuple_entry *cur;
  tuple_queue *queue = &(TUPLES[type]);

  for (current = &queue->head;
      (*current) != NULL;
      current = &(*current)->next) {
    cur = *current;

    size_t sizeBegin = TYPE_FIELD_SIZE + TYPE_ARG_OFFSET(type, field_aggregate);
    char *start = (char*)(cur->tuple);

    if(memcmp(start, tuple, sizeBegin))
      continue;

    size_t sizeOffset = sizeBegin + TYPE_ARG_SIZE(type, field_aggregate);
    size_t sizeEnd = TYPE_SIZE(type) - sizeOffset;

    if (memcmp(start + sizeOffset, (char*)tuple + sizeOffset, sizeEnd))
      continue;
    tuple_queue *agg_queue = cur->records.agg_queue;

    /* AGG_FIRST aggregate optimization */
    if(AGG_AGG(type_aggregate) == AGG_FIRST
        && isNew > 0
        && !queue_is_empty(agg_queue)) {
      FREE_TUPLE(tuple);
      return;
    }

    tuple_entry** current2;
    tuple_entry* cur2;

    for (current2 = &agg_queue->head;
        *current2 != NULL;
        current2 = &(*current2)->next) {
      cur2 = *current2;

      if (memcmp(cur2->tuple, tuple, TYPE_SIZE(type)) == 0) {
        cur2->records.count += isNew;

        if (cur2->records.count <= 0) {
          // remove it
          FREE_TUPLE(queue_dequeue_pos(agg_queue, current2));

          if (queue_is_empty(agg_queue)) {
            /* aggregate is removed */
            void *aggTuple = queue_dequeue_pos(queue, current);

            /* delete queue */
            free(agg_queue);
            // asm("NOP");//("From tuple_do_handle 3\n");
            process_bytecode(aggTuple, TYPE_START(TUPLE_TYPE(aggTuple)),
                -1, NOT_LINEAR, regis, PROCESS_TUPLE);
            aggregate_free(aggTuple, field_aggregate, AGG_AGG(type_aggregate));
            FREE_TUPLE(aggTuple);
          } else
            aggregate_recalc(cur, regis, false);
        } else
          aggregate_recalc(cur, regis, false);
#ifdef DEBUG_INSTRS
        fasm("NOP");//(stdout,
            "\x1b[1;32m--%d--\tAgg delete Iter success for %s\x1b[0m\n",
            getBlockId(), tuple_names[type]);
#endif

        FREE_TUPLE(tuple);
        return;
      }
    }

    // if deleting, return
    if (isNew <= 0) {
#ifdef DEBUG_INSTRS
      fasm("NOP");//(stdout,
          "\x1b[1;32m--%d--\tAgg delete Iter failure for %s\x1b[0m\n",
          getBlockId(), tuple_names[type]);
#endif

      FREE_TUPLE(tuple);
      return;
    }
    record_type newRecord;
    newRecord.count = isNew;
    newRecord.agg_queue = NULL;
    queue_enqueue(agg_queue, tuple, newRecord);
    aggregate_recalc(cur, regis, false);

    return;
  }

  // if deleting, return
  if (isNew <= 0) {
    FREE_TUPLE(tuple);
    return;
  }

  // So now we know we have a new tuple
  tuple_t tuple_cpy = ALLOC_TUPLE(TYPE_SIZE(type));
  memcpy(tuple_cpy, tuple, TYPE_SIZE(type));

  /* create aggregate queue */
  tuple_queue *agg_queue = (tuple_queue*)malloc(sizeof(tuple_queue));

  queue_init(agg_queue);

  record_type newRecord;
  newRecord.count = isNew;
  newRecord.agg_queue = NULL;
  queue_enqueue(agg_queue, tuple, newRecord);

  record_type newRecord1;
  newRecord1.agg_queue = agg_queue;
  tuple_entry *entry = queue_enqueue(&TUPLES[type], tuple_cpy, newRecord1);

  aggregate_recalc(entry, regis, true);
  // asm("NOP");//("From tuple_do_handle 4\n");
  process_bytecode(tuple, TYPE_START(type), isNew, NOT_LINEAR, regis, PROCESS_TUPLE);
}

NodeID getBlockId (void) { return blockId; }

inline byte val_is_float(const byte x) {
    return x == 0x00;
  }
  inline byte val_is_int(const byte x) {
    return x == 0x01;
  }
  inline byte val_is_field(const byte x) {
    return x == 0x02;
  }

void __myassert(char * file, int line, char * exp) {
#ifdef LOG_DEBUG
    //{
    char str[50];
    sasm("NOP");//(str, "myassert %s:%d %s", file, line, exp);
    printDebug(str);
    //}
#endif
    /*
       while (1) {
       setColor(RED);
       setColor(BLUE);
       }
       */
    // OUTPUT << "Assert " <<  " " << file << " " << line << " " << exp << endl;
  }

  /* Check if rule of ID rid is ready to be derived */
  /* Returns 1 if true, 0 otherwise */
byte updateRuleState(byte rid) {
  int i;
  /* A rule is ready if all included predicates are present in the database */
  for (i = 0; i < RULE_NUM_INCLPREDS(rid); ++i) {
    if (TUPLES[RULE_INCLPRED_ID(rid, i)].length == 0)
      return INACTIVE_RULE;
  }

  /* Rule is ready, enqueue it or process it rightaway */
  return ACTIVE_RULE;
}

/* ************* AGGREGATE RELATED FUNCTIONS ************* */

inline bool aggregate_accumulate(int agg_type, void *acc, void *obj, int count) {
    switch (agg_type) {
      case AGG_SET_UNION_INT:
      case AGG_SET_UNION_FLOAT:
        myassert(false);
        return false;

      case AGG_FIRST:
        return false;

      case AGG_MAX_INT:
        if (MELD_INT(obj) > MELD_INT(acc)) {
          MELD_INT(acc) = MELD_INT(obj);
          return true;
        } else
          return false;

      case AGG_MIN_INT:
        if (MELD_INT(obj) < MELD_INT(acc)) {
          MELD_INT(acc) = MELD_INT(obj);
          return true;
        } else
          return false;

      case AGG_SUM_INT:
        MELD_INT(acc) += MELD_INT(obj) * count;
        return false;

      case AGG_MAX_FLOAT:
        if(MELD_FLOAT(obj) > MELD_FLOAT(acc)) {
          MELD_FLOAT(acc) = MELD_FLOAT(obj);
          return true;
        } else
          return false;

      case AGG_MIN_FLOAT:
        if(MELD_FLOAT(obj) < MELD_FLOAT(acc)) {
          MELD_FLOAT(acc) = MELD_FLOAT(obj);
          return true;
        } else
          return false;

      case AGG_SUM_FLOAT:
        MELD_FLOAT(acc) += MELD_FLOAT(obj) * (meld_float)count;
        return false;

      case AGG_SUM_LIST_INT:
      case AGG_SUM_LIST_FLOAT:
        myassert(false);
        return false;
    }

    myassert(0);
    while(1);
  }

  inline bool aggregate_changed(int agg_type, void *v1, void *v2) {
    switch(agg_type) {
      case AGG_FIRST:
        return false;

      case AGG_MIN_INT:
      case AGG_MAX_INT:
      case AGG_SUM_INT:
        return MELD_INT(v1) != MELD_INT(v2);

      case AGG_MIN_FLOAT:
      case AGG_MAX_FLOAT:
      case AGG_SUM_FLOAT:
        return MELD_FLOAT(v1) != MELD_FLOAT(v2);

      case AGG_SET_UNION_INT:
      case AGG_SET_UNION_FLOAT:
        return false;

      case AGG_SUM_LIST_INT:
      case AGG_SUM_LIST_FLOAT:
        return false;

      default:
        myassert(0);
        return true;
    }

    myassert(0);
    while(1);
  }

  inline void aggregate_seed(int agg_type, void *acc, void *start, int count, size_t size) {
    switch(agg_type) {
      case AGG_FIRST:
        memcpy(acc, start, size);
        return;
      case AGG_MIN_INT:
      case AGG_MAX_INT:
        MELD_INT(acc) = MELD_INT(start);
        return;
      case AGG_SUM_INT:
        MELD_INT(acc) = MELD_INT(start) * count;
        return;
      case AGG_MIN_FLOAT:
      case AGG_MAX_FLOAT:
        MELD_FLOAT(acc) = MELD_FLOAT(start);
        return;
      case AGG_SUM_FLOAT:
        MELD_FLOAT(acc) = MELD_FLOAT(start) * count;
        return;
      case AGG_SET_UNION_INT:
      case AGG_SET_UNION_FLOAT:
      case AGG_SUM_LIST_INT:
      case AGG_SUM_LIST_FLOAT:
        myassert(false);
        return;
    }

    myassert(0);
    while(1);
  }

inline void aggregate_free(tuple_t tuple, unsigned char field_aggregate, unsigned char type_aggregate) {
    switch(type_aggregate) {
      case AGG_FIRST:
      case AGG_MIN_INT:
      case AGG_MAX_INT:
      case AGG_SUM_INT:
      case AGG_MIN_FLOAT:
      case AGG_MAX_FLOAT:
      case AGG_SUM_FLOAT:
        /* nothing to do */
        break;

      case AGG_SET_UNION_INT:
      case AGG_SET_UNION_FLOAT:
      case AGG_SUM_LIST_INT:
      case AGG_SUM_LIST_FLOAT:
        myassert(false);
        break;

      default:
        myassert(0);
        break;
    }
  }

  inline void aggregate_recalc(tuple_entry *agg, Register *regis, bool first_run) {
    tuple_type type = TUPLE_TYPE(agg->tuple);

    tuple_entry *cur;

    int agg_type = AGG_AGG(TYPE_AGGREGATE(type));
    int agg_field = AGG_FIELD(TYPE_AGGREGATE(type));
    tuple_queue *agg_queue = agg->records.agg_queue;
    tuple_entry *agg_list = agg_queue->head;
    tuple_t tuple = agg_list->tuple;

    void* start = GET_TUPLE_FIELD(tuple, agg_field);

    /* make copy */
    size_t size = TYPE_ARG_SIZE(type, agg_field);
    void* accumulator = malloc(size);

    aggregate_seed(agg_type, accumulator, start, agg_list->records.count, size);

    /* calculate offsets to copy right side to aggregated tuple */
    size_t size_offset = TYPE_FIELD_SIZE + TYPE_ARG_OFFSET(type, agg_field) + TYPE_ARG_SIZE(type, agg_field);
    size_t total_copy = TYPE_SIZE(type) - size_offset;
    tuple_t target_tuple = NULL;

    if (total_copy > 0)
      target_tuple = tuple;

    for (cur = agg_list->next; cur != NULL; cur = cur->next) {
      if(aggregate_accumulate(agg_type, accumulator,
            GET_TUPLE_FIELD(cur->tuple, agg_field), cur->records.count))
        target_tuple = cur->tuple;
    }

    void *acc_area = GET_TUPLE_FIELD(agg->tuple, agg_field);

    if(first_run)
      memcpy(acc_area, accumulator, size);
    else if (aggregate_changed(agg_type, acc_area, accumulator)) {
      // asm("NOP");//("From aggregate_recalc 1\n");
      process_bytecode(agg->tuple, TYPE_START(type), -1, NOT_LINEAR, regis, PROCESS_TUPLE);
      aggregate_free(agg->tuple, agg_field, agg_type);
      memcpy(acc_area, accumulator, size);
      if (total_copy > 0) /* copy right side from target tuple */
        memcpy(((unsigned char *)agg->tuple) + size_offset, ((unsigned char *)target_tuple) + size_offset, total_copy);
      // asm("NOP");//("From aggregate_recalc 2\n");
      process_bytecode(agg->tuple, TYPE_START(type), 1, NOT_LINEAR, regis, PROCESS_TUPLE);
    }

    free(accumulator);
  }

/* ************* INSTRUCTION EXECUTION FUNCTIONS ************* */

  /* Allocates a new tuple of type 'type' and set its type byte */
  inline void execute_alloc (const unsigned char *pc, Register *regis) {
    // asm("NOP");//("execute alloc %#x ",*(const unsigned char*)pc);
    ++pc;
    // asm("NOP");//("execute alloc %#x ",*(const unsigned char*)pc);
    tuple_type type = FETCH(pc++);
    byte reg_index = FETCH(pc);
    // asm("NOP");//("execute alloc %#x\n",*(const unsigned char*)pc);
    tuple_t *dst = (void**)eval_reg (reg_index, &pc, regis);
    *dst = ALLOC_TUPLE(TYPE_SIZE(type));
#if defined(DEBUG_INSTRS) || defined(DEBUG_ALLOCS)
    {
      asm("NOP");// ("--%d--\t ALLOC %s TO regis %d\n",
          getBlockId(), tuple_names[type], reg_index);
    }
#endif
    memset (*dst, 0, TYPE_SIZE(type));
    TUPLE_TYPE(*dst) = type;
  }

  /* Either enqueue a tuple for derivation
   * or enqueue it for retraction
   */
  inline void execute_addtuple (const unsigned char *pc, Register *regis, int isNew) {
    ++pc;
    byte reg_index = FETCH(pc);
    Register tuple_reg = regis[reg_index];

#ifdef DEBUG_INSTRS
    tuple_type type = TUPLE_TYPE((tuple_t)tuple_reg);
    if (isNew < 0)
      asm("NOP");// ("--%d--\t Enqueue fact regis %d: %s\n",
          getBlockId(), reg_index, tuple_names[type]);
    else
      asm("NOP");// ("--%d--\t Enqueue RETRACTION fact regis %d: %s\n",
          getBlockId(), reg_index, tuple_names[type]);
#endif
    record_type newRecord;
    newRecord.count = isNew;
    enqueueNewTuple((tuple_t)MELD_CONVERT_REG_TO_PTR(tuple_reg), newRecord);
  }

  /* Only used to notify user that a linear tuple has been updated
   * instead of removed during rule derivation.
   */
  inline void execute_update (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg_index = FETCH(pc);
    Register tuple_reg = regis[reg_index];

#ifdef DEBUG_INSTRS
    tuple_type type = TUPLE_TYPE((tuple_t)tuple_reg);
    asm("NOP");// ("--%d--\t UPDATE regis %d: %s\n", getBlockId(), reg_index,
        tuple_names[type]);
#else
    (void)tuple_reg;
#endif
  }

  /* Send tuple pointed at by 'send_reg' to blockID designated by send_rt
   * NO DELAY!
   */
  inline void execute_send (const unsigned char *pc, Register *regis, int isNew) {
    ++pc;
    Register send_reg = regis[SEND_MSG(pc)];
    NodeID send_rt = regis[SEND_RT(pc)];

#ifdef DEBUG_INSTRS
    asm("NOP");//("--%d--\t SEND regis %d TO regis %d\n",
        getBlockId(), SEND_MSG(pc), SEND_RT(pc));
#endif

    tuple_send((tuple_t)MELD_CONVERT_REG_TO_PTR(send_reg), send_rt, 0, isNew);
  }

  /* Call an external function with one argument.
   * Not implemented yet, only used to support the node2int function
   * which is useless when using BB as nodeID's are not pointers.
   */
  inline void execute_call1 (const unsigned char *pc, Register *regis) {
    ++pc;
    byte functionID = FETCH(pc++);

    byte dst_index = FETCH(pc);
    Register *dst = (Register*)eval_reg (dst_index, &pc, regis);

    byte return_type = FETCH(pc++);
    byte garbage_collected = FETCH(pc++);

    byte arg1_index = FETCH(pc);
    Register *arg1 = (Register*)eval_reg (arg1_index, &pc, regis);

#ifdef DEBUG_INSTRS
    if (functionID == NODE2INT_FUNC)
      /* No need to do anything for this function since VM is already *
       * considering node args as NodeID's, which are int's           */
      asm("NOP");//("--%d--\t CALL1 node2int/%d TO regis %d = (regis %d)\n",
          getBlockId(), arg1_index, dst_index, arg1_index);
    else
      asm("NOP");//("--%d--\t CALL1 (some func)/%d TO regis %d = (regis %d)\n",
          getBlockId(), arg1_index, dst_index, arg1_index);
#endif
    if (functionID == NODE2INT_FUNC) {
      *dst = MELD_NODE_ID(arg1);
    } else {
      //fasm("NOP");//(stderr, "--%d--\t Error: call to function not implemented yet!\n",
          //getBlockId());
    }

    /* Do nothing for now since no function are currently implemented */
    (void)arg1;
    (void)dst;
    (void)return_type;
    (void)garbage_collected;
  }

  /* Similar to send, but with a delay */
  inline void execute_send_delay (const unsigned char *pc, Register *regis, int isNew) {
    ++pc;

    const byte tpl = SEND_MSG(pc);
    const byte dst = SEND_RT(pc);
    Register send_reg = regis[tpl];
    NodeID send_rt = regis[dst];
    pc += 2;
    meld_int *delay = (meld_int*)eval_int(&pc);

#ifdef DEBUG_INSTRS
    asm("NOP");//("--%d--\t SEND regis %d TO regis %d(%d) WITH DELAY %dms\n",
        getBlockId(), SEND_MSG(pc), SEND_RT(pc), send_rt, *delay);
#endif

    if(tpl == dst) {
      tuple_send((tuple_t)MELD_CONVERT_REG_TO_PTR(send_reg), getBlockId(), *delay, isNew);
    } else {
      tuple_send((tuple_t)MELD_CONVERT_REG_TO_PTR(send_reg), send_rt, *delay, isNew);
    }
  }

  /* Run an action onto the block */
  inline void execute_run_action (const unsigned char *pc, Register *regis, int isNew) {
    ++pc;

    byte reg_index = FETCH(pc);

    tuple_t action_tuple = (tuple_t)regis[reg_index];
    tuple_type type = TUPLE_TYPE(action_tuple);
    // asm("NOP");//("From execute_run_action\n");
    execute_run_action0(action_tuple, type, isNew);
  }

  inline void execute_remove (const unsigned char *pc, Register *regis, int isNew) {
    if (isNew > 0) {
      ++pc;
      int reg_remove = REMOVE_REG(pc);
      tuple_type type = TUPLE_TYPE(MELD_CONVERT_REG_TO_PTR(regis[reg_remove]));
      int size = TYPE_SIZE(type);

#ifdef DEBUG_INSTRS
      asm("NOP");// ("--%d--\t REMOVE regis %d: %s of size %d\n",
          getBlockId(), reg_remove, tuple_names[type], size);
#endif

      tuple_handle(memcpy(malloc(size),MELD_CONVERT_REG_TO_PTR(regis[reg_remove]), size), -1, regis);
      regis[REMOVE_REG(pc)] = 0;
    }
  }

  /* ************* MOVE INSTRUCTIONS FUNCTIONS ************* */

  /* Moves an int to a tuple field */
  inline void execute_mvintfield (const unsigned char *pc, Register *regis) {
    ++pc;

    Register *src = (Register*)eval_int (&pc);
    byte reg_index = FETCH(pc+1);
    byte field_num = FETCH(pc);

    tuple_t dst_tuple = (tuple_t)regis[reg_index];
    tuple_type type = TUPLE_TYPE(dst_tuple);

    Register *dst = (Register*)eval_field (dst_tuple, &pc);


#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t MOVE INT %d TO FIELD %d.%d\n",
        getBlockId(), MELD_INT(src), reg_index, field_num);
#endif

    size_t size = TYPE_ARG_SIZE(type, field_num);

    memcpy(dst, src, size);
  }

  /* Moves pointer to an int to a register */
  inline void execute_mvintreg (const unsigned char *pc, Register *regis) {
    ++pc;

    Register *src = (Register*)eval_int (&pc);
    byte reg_index = FETCH(pc);
    Register *dst = (Register*)eval_reg (reg_index, &pc, regis);

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t MOVE INT %d TO regis %d\n",
        getBlockId(), MELD_INT(src), reg_index);
#endif
    size_t size = sizeof(Register);
    memcpy(dst, src, size);
  }

  /* Moves pointer to a float to a register */
  inline void execute_mvfloatreg (const unsigned char *pc, Register *regis) {
    ++pc;

    Register *src = (Register*)eval_float (&pc);
    byte reg_index = FETCH(pc);
    Register *dst = (Register*)eval_reg (reg_index, &pc, regis);

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t MOVE FLOAT %f TO regis %d\n",
        getBlockId(), MELD_FLOAT(src), reg_index);
#endif

    size_t size = sizeof(Register);
    memcpy(dst, src, size);
  }

  /* Moves pointer to a float to a tuple field */
  inline void execute_mvfloatfield (const unsigned char *pc, Register *regis) {
    ++pc;

    Register *src = (Register*)eval_float (&pc);
    byte reg_index = FETCH(pc+1);
    byte field_num = FETCH(pc);

    tuple_t dst_tuple = (tuple_t)regis[reg_index];
    tuple_type type = TUPLE_TYPE(dst_tuple);

    Register *dst = (Register*)eval_field (dst_tuple, &pc);


#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t MOVE FLOAT %f TO FIELD %d.%d\n",
        getBlockId(), MELD_FLOAT(src), reg_index, field_num);
#endif

    size_t size = TYPE_ARG_SIZE(type, field_num);

    memcpy(dst, src, size);
  }

  /* Moves pointer to a tuple field to a register */
  inline void execute_mvfieldreg (const unsigned char *pc, Register *regis) {
    ++pc;
    byte field_reg = FETCH(pc+1);
    byte field_num = FETCH(pc);

    tuple_t tpl = (tuple_t)regis[field_reg];
    Register *src = (Register*)eval_field (tpl, &pc);

    byte reg_index = FETCH(pc);
    Register *dst = (Register*)eval_reg (reg_index, &pc, regis);

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t MOVE FIELD %d.%d TO regis %d\n",
        getBlockId(), field_reg, field_num,
        reg_index);
#else
    (void)field_num;
#endif
    size_t size = TYPE_ARG_SIZE(TUPLE_TYPE(tpl), field_num);
    memcpy(dst, src, size);
  }

  /* Moves value pointed at by a register to a field */
  inline void execute_mvregfield (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg_index = FETCH(pc);
    Register *src = (Register*)eval_reg (reg_index, &pc, regis);

    byte field_reg = FETCH(pc+1);
    byte field_num = FETCH(pc);

    tuple_t field_tpl = (tuple_t)regis[field_reg];
    tuple_type type = TUPLE_TYPE(field_tpl);
    Register *dst = (Register*)eval_field (field_tpl, &pc);

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t MOVE REG %d TO FIELD %d.%d\n",
        getBlockId(), reg_index, field_reg, field_num);
#endif

    size_t size = TYPE_ARG_SIZE(type, field_num);

    memcpy(dst, src, size);
  }

  /* Moves content of a tuple field to another */
  inline void execute_mvfieldfield (const unsigned char *pc, Register *regis) {
    ++pc;
    byte src_field_reg = FETCH(pc+1);
    byte src_field_num = FETCH(pc);

    tuple_t src_field_tpl = (tuple_t)regis[src_field_reg];
    Register *src = (Register*)eval_field (src_field_tpl, &pc);

    byte dst_field_reg = FETCH(pc+1);
    byte dst_field_num = FETCH(pc);

    tuple_t dst_field_tpl = (tuple_t)regis[dst_field_reg];
    tuple_type type = TUPLE_TYPE(dst_field_tpl);
    Register *dst = (Register*)eval_field (dst_field_tpl, &pc);

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t MOVE FIELD %d.%d TO FIELD %d.%d\n",
        getBlockId(), src_field_reg, src_field_num,
        dst_field_reg, dst_field_num);
#else
    (void) src_field_num;
#endif

    size_t size = TYPE_ARG_SIZE(type, dst_field_num);

    memcpy(dst, src, size);
  }

  /* Moves blockId to a tuple field */
  inline void execute_mvhostfield (const unsigned char *pc, Register *regis) {
    ++pc;

    Register *src = (Register*)EVAL_HOST;

    byte field_reg = FETCH(pc+1);
    byte field_num = FETCH(pc);

    tuple_t field_tpl = (tuple_t)regis[field_reg];
    tuple_type type = TUPLE_TYPE(field_tpl);
    Register *dst = (Register*)eval_field (field_tpl, &pc);

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t MOVE HOST TO FIELD %d.%d\n",
        getBlockId(), field_reg, field_num);
#endif

    size_t size = TYPE_ARG_SIZE(type, field_num);

    memcpy(dst, src, size);
  }

  /* Moves blockId to a register */
  inline void execute_mvhostreg (const unsigned char *pc, Register *regis) {
    ++pc;

    Register *src = (Register*)EVAL_HOST;

    byte reg_index = FETCH(pc);
    Register *dst = (Register*)eval_reg (reg_index, &pc, regis);

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t MOVE HOST TO regis %d\n",
        getBlockId(), reg_index);
#endif

    size_t size = sizeof(Register);

    memcpy(dst, src, size);
  }

  /* Moves content of a regis to another */
  inline void execute_mvregreg (const unsigned char *pc, Register *regis) {
    ++pc;

    byte src_reg_index = FETCH(pc);
    Register *src = (Register*)eval_reg (src_reg_index, &pc, regis);

    byte dst_reg_index = FETCH(pc);
    Register *dst = (Register*)eval_reg (dst_reg_index, &pc, regis);

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t MOVE REG %d TO REG %d\n",
        getBlockId(), src_reg_index, dst_reg_index);
#endif

    size_t size = sizeof(Register);

    memcpy(dst, src, size);
  }


/* ************* EVAL FUNCTIONS ************* */

  /* Returns the address of field number 'field_num' (extracted from byte code)
   * of the tuple given as argument.
   * Also increment pc past the field.
   */
  inline void* eval_field (tuple_t tuple, const unsigned char **pc) {
    const unsigned char field_num = VAL_FIELD_NUM(*pc);
    (*pc) += 2;

    return GET_TUPLE_FIELD(tuple, field_num);
  }
  /* Returns the address of register number 'value'
   * and increment pc past the regis byte.
   */
  inline void* eval_reg(const unsigned char value, const unsigned char **pc, Register *regis) {
    // asm("NOP");//("eval reg: %#x ", *(const unsigned char*)pc );
    ++(*pc);
    // asm("NOP");//("eval reg: %#x\n", *(const unsigned char*)pc );
    // asm("NOP");//("eval reg: %lu\n", (regis)[VAL_REG(value)] );
    return &(regis)[VAL_REG(value)];
  }

  /* Returns a pointer to the meld_int at address pointed at by pc
   * and increment it past the int.
   */
  inline void* eval_int (const unsigned char **pc) {
    void *ret = (void *)(*pc);
    *pc += sizeof(meld_int);

    return ret;
  }

  /* Returns a pointer to the meld_float (double) at address pointed at by pc
   * and increment it past the float.
   */
  inline void* eval_float (const unsigned char **pc) {
    void *ret = (void *)(*pc);
    *pc += sizeof(meld_float);

    return ret;
  }

  /* Set value of register number 'reg_index' as a pointer to tuple 'tuple' */
  inline void moveTupleToReg (const unsigned char reg_index, tuple_t tuple, Register *regis) {
    Register *dst = &(regis)[VAL_REG(reg_index)];
    *dst = (Register)tuple;

#if 0
    asm("NOP");// ("--%d--\t MOVE %s to regis %d\n", getBlockId(),
        tuple_names[TUPLE_TYPE(regis[reg_index])], reg_index);
#endif
  }


/* ************* OPERATION INSTRUCTIONS FUNCTIONS ************* */

  /* Perform boolean NOT operation */
  inline void execute_not (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);

    Register *arg = (Register*)eval_reg (reg1, &pc, regis);
    Register *dest = (Register*)eval_reg (reg2, &pc, regis);

    if (MELD_BOOL(arg) > 0)
      *dest = 0;
    else
      *dest = 1;

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t NOT regis %d TO regis %d\n",
        getBlockId(), reg1, reg2);
#endif
  }

  /* Perform boolean OR operation */
  inline void execute_boolor (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);

    *dest = (MELD_BOOL(arg1) | MELD_BOOL(arg2));

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t BOOL regis %d OR regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_boolequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);

    *dest = (MELD_BOOL(arg1) == MELD_BOOL(arg2));

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t BOOL regis %d EQUAL regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_boolnotequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);

    *dest = (MELD_BOOL(arg1) != MELD_BOOL(arg2));

#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t BOOL regis %d NOT EQUAL regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  /* Compares two blockId and store the result to 'dest' */
  inline void execute_addrequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_NODE_ID(arg1) == MELD_NODE_ID(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t ADDR regis %d EQUAL regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  /* Compares two blockId and store the result to 'dest' */
  inline void execute_addrnotequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_NODE_ID(arg1) != MELD_NODE_ID(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t ADDR regis %d NOTEQUAL regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  /* Same with and int... */
  inline void execute_intequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) == MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d EQUAL regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_intnotequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) != MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d NOTEQUAL regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_intgreater (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) > MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d GREATER THAN regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_intgreaterequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) >= MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d GREATER/EQUAL THAN regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_intlesser (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) < MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d LESSER THAN regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_intlesserequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) <= MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d LESSER/EQUAL THAN regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_intmul (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) * MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d MULTIPLIED BY regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_intdiv (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) / MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d DIVIDED BY regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_intmod (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) % MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d MOD regis %d TO regis %d\n", getBlockId(), reg1, reg2, reg3);
#endif

  }
  inline void execute_intplus (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) + MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d PLUS regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_intminus (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_INT(arg1) - MELD_INT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t INT regis %d MINUS regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_floatplus (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_FLOAT(arg1) + MELD_FLOAT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t FLOAT regis %d PLUS regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_floatminus (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_FLOAT(arg1) - MELD_FLOAT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t FLOAT regis %d MINUS regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_floatmul (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_FLOAT(arg1) * MELD_FLOAT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t FLOAT regis %d MULTIPLIED BY regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_floatdiv (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_FLOAT(arg1) / MELD_FLOAT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t FLOAT regis %d DIVIDED BY regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_floatequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_FLOAT(arg1) == MELD_FLOAT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t FLOAT regis %d EQUAL regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_floatnotequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_FLOAT(arg1) != MELD_FLOAT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t FLOAT regis %d NOT EQUAL regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_floatlesser (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_FLOAT(arg1) < MELD_FLOAT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t FLOAT regis %d LESSER THAN regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_floatlesserequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_FLOAT(arg1) <= MELD_FLOAT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t FLOAT regis %d LESSER/EQUAL THAN regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_floatgreater (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_FLOAT(arg1) > MELD_FLOAT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t FLOAT regis %d GREATER THAN regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }

  inline void execute_floatgreaterequal (const unsigned char *pc, Register *regis) {
    ++pc;

    byte reg1 = FETCH(pc);
    byte reg2 = FETCH(pc+1);
    byte reg3 = FETCH(pc+2);

    Register *arg1 = (Register*)eval_reg (reg1, &pc, regis);
    Register *arg2 = (Register*)eval_reg (reg2, &pc, regis);
    Register *dest = (Register*)eval_reg (reg3, &pc, regis);
    *dest = (MELD_FLOAT(arg1) >= MELD_FLOAT(arg2));
#ifdef DEBUG_INSTRS
    asm("NOP");// ("--%d--\t FLOAT regis %d GREATER/EQUAL THAN regis %d TO regis %d\n",
        getBlockId(), reg1, reg2, reg3);
#endif
  }



int process_bytecode (tuple_t tuple, const unsigned char *pc, int isNew, int isLinear, Register *regis, byte state) {
  
#ifdef DEBUG_INSTRS

  /* if (PROCESS_TYPE(state) == PROCESS_TUPLE) { */
  /*     pthread_mutex_lock(&(printMutex)); */
  /*     asm("NOP");// ("\n--%d--\tPROCESS TUPLE ", getBlockId()); */
  /*     tuple_print (tuple, stdout); */
  /*     asm("NOP");// ("\n"); */
  /*     pthread_mutex_unlock(&(printMutex)); */
  /*   } */
  /* #else */

  static int called=0;
  called++;
  // asm("NOP");//("No of times process_bytecode called: %d\n", called);
  if (PROCESS_TYPE(state) == PROCESS_TUPLE)
    asm("NOP");// ("\n--%d--\tPROCESS TUPLE %s -- isNew = %d\n", getBlockId(), tuple_names[TUPLE_TYPE(tuple)], isNew);
  else{
    if (PROCESS_TYPE(state) == PROCESS_ITER) {
      asm("NOP");// ("--%d--\t PROCESS ITER %s\n", getBlockId(),
          tuple_names[TUPLE_TYPE(tuple)]);
    }

    /* Dont't print if rule is persistent */
    else if (PROCESS_TYPE(state) == PROCESS_RULE) {
      if (!RULE_ISPERSISTENT(RULE_NUMBER(state)))
        asm("NOP");// ("--%d--\t PROCESS RULE %d: %s\n", getBlockId(),
            RULE_NUMBER(state), rule_names[RULE_NUMBER(state)]);

    } else
      asm("NOP");// ("\n--%d--\tERROR: UNKNOWN PROCESS TYPE\n", getBlockId());
  }
#endif
  /* Move tuple to register 0 so it can be accessed */
  if (state == PROCESS_TUPLE)
    moveTupleToReg (0, tuple, regis);
  /* Only if process_bytecode not called by iter, */
  /* because otherwise the tuple is already in a register */

  for (;;) {
eval_loop:
#ifdef DEBUG_INSTRS
#ifdef LOG_DEBUG
    print_bytecode(pc);
#endif
#endif
    // break;
    asm("NOP");//("Inside process tuple!! %#x\n\n", *(const unsigned char*)pc);
    switch (*(const unsigned char*)pc) {
      case RETURN_INSTR: {  /* 0x0 */
#ifdef DEBUG_INSTRS
                           if (!(PROCESS_TYPE(state) == PROCESS_RULE
                                 && RULE_ISPERSISTENT(RULE_NUMBER(state))) )
                             asm("NOP");// ("--%d--\tRETURN\n", getBlockId());
#endif
                           return RET_RET;
                         }

      case NEXT_INSTR: {  /* 0x1 */
#ifdef DEBUG_INSTRS
                         asm("NOP");// ("--%d--\t NEXT\n", getBlockId());
#endif
                         return RET_NEXT;
                       }

#define DECIDE_NEXT_ITER()                   \
                       if(ret == RET_LINEAR)                  \
                       return RET_LINEAR;                  \
                       if(ret == RET_DERIVED && isLinear)     \
                       return RET_DERIVED;                 \
                       if(ret == RET_RET)                     \
                       return RET_RET;                     \
                       pc = npc; goto eval_loop;

      case PERS_ITER_INSTR: { /* 0x02 */
                              const byte *npc = pc + ITER_OUTER_JUMP(pc);
                              const int ret = execute_iter (pc, regis, isNew, isLinear);
                              // asm("NOP");//("ret2: %d\n", ret);
                              DECIDE_NEXT_ITER();
                            }

      case LINEAR_ITER_INSTR: { /* 0x05 */
                                const byte *npc = pc + ITER_OUTER_JUMP(pc);
                                const int ret = execute_iter (pc, regis, isNew, isLinear);
                                // asm("NOP");//("ret5: %d\n", ret);
                                DECIDE_NEXT_ITER();
                              }

      case NOT_INSTR: { /* 0x07 */
                        const byte *npc = pc + NOT_BASE;
                        execute_not (pc, regis);
                        pc = npc;
                        goto eval_loop;
                      }

      case SEND_INSTR: {  /* 0x08 */
                         const byte *npc = pc + SEND_BASE;
                         execute_send (pc, regis, isNew);
                         pc = npc;
                         goto eval_loop;
                       }

      case RESET_LINEAR_INSTR: { /* 0x0e */
                                // asm("NOP");//("From RESET_LINEAR_INSTR\n");
                                 int ret = process_bytecode(tuple, pc + RESET_LINEAR_BASE, isNew, NOT_LINEAR, regis, PROCESS_ITER);
                                 (void)ret;
                                 pc += RESET_LINEAR_JUMP(pc);
                                 goto eval_loop;
                               }
                               break;

      case END_LINEAR_INSTR: /* 0x0f */
                               return RET_LINEAR;

      case RULE_INSTR: {  /* 0x10 */
                         const byte *npc = pc + RULE_BASE;
#ifdef DEBUG_INSTRS
                         byte rule_number = FETCH(++pc);
                         asm("NOP");// ("--%d--\t RULE %d\n", getBlockId(),
                             rule_number);
#endif
                         pc = npc;
                         goto eval_loop;
                       }

      case RULE_DONE_INSTR: { /* 0x11 */
#ifdef DEBUG_INSTRS
                              asm("NOP");// ("--%d--\t RULE DONE\n", getBlockId());
#endif
                              const byte *npc = pc + RULE_DONE_BASE;
                              pc = npc;
                              goto eval_loop;
                            }

                            /* NOT TESTED */
      case SEND_DELAY_INSTR: {  /* 0x15 */
                               const byte *npc = pc + SEND_DELAY_BASE;
                               execute_send_delay (pc, regis, isNew);
                               pc = npc;
                               goto eval_loop;
                             }

      case RETURN_LINEAR_INSTR: {   /* 0xd0 */
#ifdef DEBUG_INSTRS
                                  asm("NOP");// ("--%d--\tRETURN LINEAR\n", getBlockId());
#endif
                                  return RET_LINEAR;
                                }

      case RETURN_DERIVED_INSTR: {    /* 0xf0 */
#ifdef DEBUG_INSTRS
                                   asm("NOP");// ("--%d--\tRETURN DERIVED\n", getBlockId());
#endif
                                   return RET_DERIVED;
                                 }

      case MVINTFIELD_INSTR: {  /* 0x1e */
                               const byte *npc = pc + MVINTFIELD_BASE;
                               execute_mvintfield (pc, regis);
                               pc = npc;
                               goto eval_loop;
                             }

      case MVINTREG_INSTR: {  /* 0x1f */
                             const byte *npc = pc + MVINTREG_BASE;
                             execute_mvintreg (pc, regis);
                             pc = npc;
                             goto eval_loop;
                           }

      case MVFIELDFIELD_INSTR: {  /* 0x21 */
                                 const byte *npc = pc + MVFIELDFIELD_BASE;
                                 execute_mvfieldfield (pc, regis);
                                 pc = npc;
                                 goto eval_loop;
                               }

      case MVFIELDREG_INSTR: {  /* 0x22 */
                               const byte *npc = pc + MVFIELDREG_BASE;
                               execute_mvfieldreg (pc, regis);
                               pc = npc;
                               goto eval_loop;
                             }

      case MVPTRREG_INSTR: {  /* 0x23 */
                             const byte *npc = pc + MVPTRREG_BASE;
#ifdef DEBUG_INSTRS
                             asm("NOP");// ("--%d--\tMOVE PTR TO REG -- Do nothing\n", getBlockId());
#endif
                             /* TODO: Do something if used elsewhere than axiom derivation */
                             pc = npc;
                             goto eval_loop;
                           }


      case MVFIELDFIELDR_INSTR: { /* 0x25 */
                                  const byte *npc = pc + MVFIELDFIELD_BASE;
                                  execute_mvfieldfield (pc, regis);
                                  pc = npc;
                                  goto eval_loop;
                                }

      case MVREGFIELD_INSTR: {  /* 0x26 */
                               const byte *npc = pc + MVREGFIELD_BASE;
                               execute_mvregfield (pc, regis);
                               pc = npc;
                               goto eval_loop;
                             }

      case MVHOSTFIELD_INSTR: { /* 0x28 */
                                const byte *npc = pc + MVHOSTFIELD_BASE;
                                execute_mvhostfield (pc, regis);
                                pc = npc;
                                goto eval_loop;
                              }

                              /* NOT TESTED */
      case MVFLOATFIELD_INSTR: {  /* 0x2d */
                                 const byte *npc = pc + MVFLOATFIELD_BASE;
                                 execute_mvfloatfield (pc, regis);
                                 pc = npc;
                                 goto eval_loop;
                               }

                               /* NOT TESTED */
      case MVFLOATREG_INSTR: {  /* 0x2e */
                               const byte *npc = pc + MVFLOATREG_BASE;
                               execute_mvfloatreg (pc, regis);
                               pc = npc;
                               goto eval_loop;
                             }

                             /* NOT TESTED */
      case MVHOSTREG_INSTR: { /* 0x37 */
                              const byte *npc = pc + MVHOSTREG_BASE;
                              execute_mvhostreg (pc, regis);
                              pc = npc;
                              goto eval_loop;
                            }

      case ADDRNOTEQUAL_INSTR: {  /* 0x38 */
                                 const byte *npc = pc + OP_BASE;
                                 execute_addrnotequal (pc, regis);
                                 pc = npc;
                                 goto eval_loop;
                               }

      case ADDREQUAL_INSTR: { /* 0x39 */
                              const byte *npc = pc + OP_BASE;
                              execute_addrequal (pc, regis);
                              pc = npc;
                              goto eval_loop;
                            }

      case INTMINUS_INSTR: {  /* 0x3a */
                             const byte *npc = pc + OP_BASE;
                             execute_intminus (pc, regis);
                             pc = npc;
                             goto eval_loop;
                           }

      case INTEQUAL_INSTR: {  /* 0x3b */
                             const byte *npc = pc + OP_BASE;
                             execute_intequal (pc, regis);
                             pc = npc;
                             goto eval_loop;
                           }

      case INTNOTEQUAL_INSTR: { /* 0x3c */
                                const byte *npc = pc + OP_BASE;
                                execute_intnotequal (pc, regis);
                                pc = npc;
                                goto eval_loop;
                              }

      case INTPLUS_INSTR: { /* 0x3d */
                            const byte *npc = pc + OP_BASE;
                            execute_intplus (pc, regis);
                            pc = npc;
                            goto eval_loop;
                          }

      case INTLESSER_INSTR: { /* 0x3e */
                              const byte *npc = pc + OP_BASE;
                              execute_intlesser (pc, regis);
                              pc = npc;
                              goto eval_loop;
                            }

      case INTGREATEREQUAL_INSTR: { /* 0x3f */
                                    const byte *npc = pc + OP_BASE;
                                    execute_intgreaterequal (pc, regis);
                                    pc = npc;
                                    goto eval_loop;
                                  }

      case ALLOC_INSTR: { /* 0x40 */
                          const byte *npc = pc + ALLOC_BASE;
                          execute_alloc (pc, regis);
                          pc = npc;
                          goto eval_loop;
                        }

                        /* NOT TESTED */
      case BOOLOR_INSTR: {  /* 0x41 */
                           const byte *npc = pc + OP_BASE;
                           execute_boolor (pc, regis);
                           pc = npc;
                           goto eval_loop;
                         }

      case INTLESSEREQUAL_INSTR: {  /* 0x42 */
                                   const byte *npc = pc + OP_BASE;
                                   execute_intlesserequal (pc, regis);
                                   pc = npc;
                                   goto eval_loop;
                                 }

      case INTGREATER_INSTR: {  /* 0x43 */
                               const byte *npc = pc + OP_BASE;
                               execute_intgreater (pc, regis);
                               pc = npc;
                               goto eval_loop;
                             }

      case INTMUL_INSTR: {  /* 0x44 */
                           const byte *npc = pc + OP_BASE;
                           execute_intmul (pc, regis);
                           pc = npc;
                           goto eval_loop;
                         }

      case INTDIV_INSTR: {  /* 0x45 */
                           const byte *npc = pc + OP_BASE;
                           execute_intdiv (pc, regis);
                           pc = npc;
                           goto eval_loop;
                         }

      case FLOATPLUS_INSTR: { /* 0x46 */
                              const byte *npc = pc + OP_BASE;
                              execute_floatplus (pc, regis);
                              pc = npc;
                              goto eval_loop;
                            }

      case FLOATMINUS_INSTR: {  /* 0x47 */
                               const byte *npc = pc + OP_BASE;
                               execute_floatminus (pc, regis);
                               pc = npc;
                               goto eval_loop;
                             }

      case FLOATMUL_INSTR: {  /* 0x48 */
                             const byte *npc = pc + OP_BASE;
                             execute_floatmul (pc, regis);
                             pc = npc;
                             goto eval_loop;
                           }

      case FLOATDIV_INSTR: {  /* 0x49 */
                             const byte *npc = pc + OP_BASE;
                             execute_floatdiv (pc, regis);
                             pc = npc;
                             goto eval_loop;
                           }

      case FLOATEQUAL_INSTR: {  /* 0x4a */
                               const byte *npc = pc + OP_BASE;
                               execute_floatequal (pc, regis);
                               pc = npc;
                               goto eval_loop;
                             }

      case FLOATNOTEQUAL_INSTR: { /* 0x4b */
                                  const byte *npc = pc + OP_BASE;
                                  execute_floatnotequal (pc, regis);
                                  pc = npc;
                                  goto eval_loop;
                                }

      case FLOATLESSER_INSTR: { /* 0x4c */
                                const byte *npc = pc + OP_BASE;
                                execute_floatlesser (pc, regis);
                                pc = npc;
                                goto eval_loop;
                              }

      case FLOATLESSEREQUAL_INSTR: {  /* 0x4d */
                                     const byte *npc = pc + OP_BASE;
                                     execute_floatlesserequal (pc, regis);
                                     pc = npc;
                                     goto eval_loop;
                                   }

      case FLOATGREATER_INSTR: {  /* 0x4e */
                                 const byte *npc = pc + OP_BASE;
                                 execute_floatgreater (pc, regis);
                                 pc = npc;
                                 goto eval_loop;
                               }

      case FLOATGREATEREQUAL_INSTR: { /* 0x4f */
                                      const byte *npc = pc + OP_BASE;
                                      execute_floatgreaterequal (pc, regis);
                                      pc = npc;
                                      goto eval_loop;
                                    }

                                    /* NOT TESTED */
      case MVREGREG_INSTR: {  /* 0x50 */
                             const byte *npc = pc + MVREGREG_BASE;
                             execute_mvregreg (pc, regis);
                             pc = npc;
                             goto eval_loop;
                           }

                           /* NOT TESTED */
      case BOOLEQUAL_INSTR: { /* 0x51 */
                              const byte *npc = pc + OP_BASE;
                              execute_boolequal (pc, regis);
                              pc = npc;
                              goto eval_loop;
                            }

                            /* NOT TESTED */
      case BOOLNOTEQUAL_INSTR: {  /* 0x51 */
                                 const byte *npc = pc + OP_BASE;;
                                 execute_boolnotequal (pc, regis);
                                 pc = npc;
                                 goto eval_loop;
                               }

      case IF_INSTR: {  /* 0x60 */
                       const byte *npc = pc + IF_BASE;
                       byte *base = (byte*)pc;
                       ++pc;
                       byte reg_index = FETCH(pc);
                       Register *if_reg = (Register*)eval_reg (reg_index, &pc, regis);

                       if (!(unsigned char)(*if_reg)) {
#ifdef DEBUG_INSTRS
                         asm("NOP");// ("--%d--\t IF (regis %d) -- Failed\n",
                             getBlockId(), reg_index);
#endif
                         asm("NOP");//("if jump\n");
                         pc = base + IF_JUMP(pc);
                         goto eval_loop;
                       }
                       /* else process if content */
#ifdef DEBUG_INSTRS
                       asm("NOP");// ("--%d--\t IF (regis %d) -- Success\n",
                           getBlockId(), reg_index);
#endif
                       asm("NOP");//("no if jump\n");
                       pc = npc;
                       goto eval_loop;
                     }

      case CALL1_INSTR: { /* 0x69 */
                          const byte *npc = pc + CALL1_BASE;
                          execute_call1 (pc, regis);
                          pc = npc;
                          goto eval_loop;
                        }

      case ADDLINEAR_INSTR: { /* 0x77 */
                              const byte *npc = pc + ADDLINEAR_BASE;
                              execute_addtuple (pc, regis, isNew);
                              pc = npc;
                              goto eval_loop;
                            }

      case ADDPERS_INSTR: { /* 0x78 */
                            const byte *npc = pc + ADDPERS_BASE;
                            execute_addtuple (pc, regis, isNew);
                            pc = npc;
                            goto eval_loop;
                          }

      case RUNACTION_INSTR: { /* 0x79 */
                              const byte *npc = pc + RUNACTION_BASE;
                              // asm("NOP");//("From RUNACTION_INSTR\n");
                              execute_run_action (pc, regis, isNew);
                              pc = npc;
                              goto eval_loop;
                            }

      case UPDATE_INSTR: {  /* 0x7b */
                           const byte *npc = pc + UPDATE_BASE;
                           if (PROCESS_TYPE(state) == PROCESS_ITER)
                             execute_update (pc, regis);
                           pc = npc;
                           goto eval_loop;
                         }

      case REMOVE_INSTR: {  /* 0x80 */
                           const byte *npc = pc + REMOVE_BASE;
                           execute_remove (pc, regis, isNew);
                           pc = npc;
                           goto eval_loop;
                         }

                         /* NOT TESTED */
                         /* CAUTION: I have no way to ensure that it is the correct way to handle
                          * this instruction at this moment, please review this when you encounter it.
                          */
      case IF_ELSE_INSTR: { /* 0x81 */
                            const byte *npc = pc + IF_ELSE_BASE;
                            byte *base = (byte*)pc;
                            ++pc;
                            byte reg_index = FETCH(pc);
                            Register *if_reg = (Register*)eval_reg (reg_index, &pc, regis);

                            /* If false, jump to else */
                            if (!(unsigned char)(*if_reg)) {
#ifdef DEBUG_INSTRS
                              asm("NOP");// ("--%d--\t IF_ELSE (regis %d) -- ELSE\n",
                                  getBlockId(), reg_index);
#endif

                              pc = base + IF_JUMP(pc);
                              goto eval_loop;
                            } else {
                              /* Else, process if until a jump instruction is encountered
                               * (it seems...)
                               */
#ifdef DEBUG_INSTRS
                              asm("NOP");// ("--%d--\t IF_ELSE (regis %d) -- ELSE\n",
                                  getBlockId(), reg_index);
#endif

                              pc = npc;
                              goto eval_loop;
                            }
                          }

                          /* NOT TESTED */
      case JUMP_INSTR: {
                         ++pc;
#ifdef DEBUG_INSTRS
                         asm("NOP");// ("--%d--\t JUMP TO\n", getBlockId());
#endif
                         pc += JUMP_BASE + IF_JUMP(pc);
                         goto eval_loop;
                       }

      case INTMOD_INSTR: {  /* 0x7d */
                           const byte *npc = pc + OP_BASE;
                           execute_intmod (pc, regis);
                           pc = npc;
                           goto eval_loop;
                         }

      default:
                         asm("NOP");// ("--%d--\t "
                             //"INSTRUCTION NOT IMPLEMENTED YET: %#x %#x %#x %#x %#x\n",
                             //getBlockId(),
                             //(unsigned char)*pc, (unsigned char)*(pc+1),
                             //(unsigned char)*(pc+2), (unsigned char)*(pc+3),
                             //(unsigned char)*(pc+4));

                         exit(-2);
    }
  }
}
/* ************* QUEUE MANAGEMENT FUNCTIONS ************* */
int queue_length (tuple_queue *queue) {
    int i;
    tuple_entry *entry = queue->head;

    for (i = 0; entry != NULL; entry = entry->next, i++);

    return i;
  }

  bool queue_is_empty(tuple_queue *_queue) {
    return _queue->head == NULL;
  }

  void queue_push_tuple(tuple_queue *queue, tuple_entry *entry) {
    if(queue->head == NULL){
      queue->head = entry;
      queue->tail = entry;
    }
    else {
      queue->tail->next = entry;
      queue->tail = entry;
    }
  }

tuple_t queue_pop_tuple(tuple_queue *_queue) {
  tuple_entry *entry = NULL;

  if (_queue->head) {
    entry = _queue->head;
    _queue->head = _queue->head->next;

    if (_queue->head == NULL)
      _queue->tail = NULL;
  }

  return entry;
}

tuple_entry* queue_enqueue(tuple_queue *_queue, tuple_t tuple, record_type isNew) {
  tuple_entry *entry = (tuple_entry*)malloc(sizeof(tuple_entry));
  entry->tuple = tuple;
  entry->records = isNew;
  entry->next = NULL;
  queue_push_tuple(_queue, entry);
  _queue->length++;
  return entry;
}

tuple_t queue_dequeue(tuple_queue *queue, int *isNew) {
  tuple_entry *entry = (tuple_entry*)queue_pop_tuple(queue);
  queue->length--;

  tuple_t tuple = entry->tuple;

  if(isNew)
    *isNew = entry->records.count;

  free(entry);

  return tuple;
}

tuple_t queue_dequeue_pos(tuple_queue *_queue, tuple_entry **pos) {
    tuple_entry *entry = *pos;
    tuple_entry *next = (*pos)->next;
    _queue->length--;

    if (entry == _queue->tail) {
      if(entry == _queue->head)
        _queue->tail = NULL;
      else
        _queue->tail = (tuple_entry *)pos; /* previous */
    }

    *pos = next;

    tuple_t tuple = entry->tuple;
    free(entry);

    return tuple;
  }
tuple_pentry* p_dequeue(tuple_pqueue *q) {
    tuple_pentry *ret = q->queue;

    if(q->queue != NULL)
      q->queue = q->queue->next;

    return ret;
  }

void p_enqueue(tuple_pqueue *queue, Time priority, tuple_t tuple,
      NodeID rt, record_type isNew) {
    tuple_pentry *entry = (tuple_pentry*)malloc(sizeof(tuple_pentry));

    entry->tuple = tuple;
    entry->records = isNew;
    entry->priority = priority;
    entry->rt = rt;

    tuple_pentry **spot;
    for (spot = &(queue->queue);
        *spot != NULL &&
        (*spot)->priority <= priority;
        spot = &((*spot)->next));

    entry->next = *spot;
    *spot = entry;
  }

/* Simply calls tuple_do_handle located in core.c to handle tuple  */
void tuple_handle(tuple_t tuple, int isNew, Register *registers) {
  tuple_type type = TUPLE_TYPE(tuple);
  myassert (type < NUM_TYPES);
  tuple_do_handle(type, tuple, isNew, registers);
}

void enqueue_init(void) {
  if(TYPE_INIT == -1)
    return;

  tuple_t tuple = tuple_alloc(TYPE_INIT);

  record_type newRecord;
  newRecord.count = 1;
  newRecord.agg_queue = NULL;
  enqueueNewTuple(tuple, newRecord);

}


/*********************** THE MAIN VM FUNCTION TO PROCESS A RULE ********************/
void processOneRule(void) {
  // processing new facts and updating axioms
  waiting = 0;

  //If there are new tuples
  if(!queue_is_empty(newTuples)) {
    asm("NOP");//("I'm in \n");
    int isNew = 0;
    print_newTuples();
    tuple_t tuple = queue_dequeue(newTuples, &isNew);
    tuple_handle(tuple, isNew, regis);
    waiting = 1;
    //Else if there are new delayed tuple available
  }
  else if (!p_empty(delayedTuples) && p_peek(delayedTuples)->priority <= myGetTime()) {
    tuple_pentry *entry = p_dequeue(delayedTuples);

    tuple_send(entry->tuple, entry->rt, 0, entry->records.count);
    free(entry);
    waiting = 1;
    //Else if there are new stratified tuple
  } else if (!(p_empty(newStratTuples))) {
    tuple_pentry *entry = p_dequeue(newStratTuples);
    tuple_handle(entry->tuple, entry->records.count, regis);

    free(entry);
    //Else if there are no tuple to process
    waiting = 1;
  } else {

    if (!p_empty(delayedTuples)) {
      waiting = 1;
    }
    /* If all tuples have been processed
     * update rule state and process them if they are ready */
    for (int i = 0; i < NUM_RULES; ++i) {
      //If a rule has all its predicate (considered ACTIVE)
      if (updateRuleState(i)) {
        waiting = 1;
        /* Set state byte used by DEBUG */
        byte processState = PROCESS_RULE | (i << 4);
        /* Don't process persistent rules (which is useless)
         * as they all have only a RETURN instruction.
         */
        if (!RULE_ISPERSISTENT(i)) {
          /* Trigger execution */
          // asm("NOP");//("From processOneRule\n");
          process_bytecode (NULL, RULE_START(i), 1, NOT_LINEAR, regis, processState);

           // After one rule is executed we set the VM on waiting until next call of scheduler
          i = NUM_RULES;
        }
      }
      /* else: Rule not ready yet, set status to not waiting until new fact appear */
    }
  }

  //updateAccel();

    // update axioms based upon any changes
    int newNumNeighbors = getNeighborCount();
    if (newNumNeighbors != numNeighbors) {
      enqueue_count(numNeighbors, -1);
      numNeighbors = newNumNeighbors;
      waiting = 1;
      enqueue_count(numNeighbors, 1);
    }

    for (int i = 0; i < NUM_PORTS; i++) {
      NodeID neighbor = get_neighbor_ID(i);

      if (neighbor == neighbors[i])
        continue;

      waiting = 1;
      enqueue_face(neighbors[i], i, -1);

      /* Delete received tuples from database
       * This may need to be reviewed,
       * I am not sure what LM is supposed to do with received tuples
       */
      while(!queue_is_empty(&(receivedTuples[i]))) {
        tuple_t tuple = queue_dequeue(&receivedTuples[i], NULL);
        asm("NOP");//("--%d--\tDelete received ", blockId);
        record_type newRecord;
        newRecord.count = -1;
        enqueueNewTuple(tuple, newRecord);
      }

      neighbors[i] = neighbor;
      enqueue_face(neighbors[i], i, 1);
    }
}

bool isWaiting(){
  return waiting > 0;
}

/* Called upon block init (block.bb)
 * to ensure that data structures are allocated before
 * VM start in case other blocks send us tuples - Would seg fault otherwise */
void vm_alloc(void) {

  // init stuff
  tuples = (tuple_queue*)calloc(NUM_TYPES, sizeof(tuple_queue));
  newTuples = (tuple_queue*)calloc(1, sizeof(tuple_queue));
  newStratTuples = (tuple_pqueue*)calloc(1, sizeof(tuple_pqueue));
  delayedTuples = (tuple_pqueue*)calloc(1, sizeof(tuple_pqueue));

  myassert(tuples!=NULL);
  myassert(newTuples!=NULL);
  myassert(newStratTuples!=NULL);
  myassert(delayedTuples!=NULL);

  /* Reset received tuples queue */
  memset(receivedTuples, 0, sizeof(tuple_queue) * NUM_PORTS);

}

/* ************* VM INITIALIZATION FUNCTIONS ************* */

static int type;
void init_fields(void) {
  size_t total = 2*NUM_TYPES;
  int i, j;

  for(i = 0; i < NUM_TYPES; ++i)
    total += TYPE_NUMARGS(i) * 2;

  arguments = (unsigned char*)malloc(total);
  unsigned char *start = arguments + 2*NUM_TYPES;
  unsigned char offset, size;

  for(i = 0; i < NUM_TYPES; ++i) {
    arguments[i*2] = start - arguments; /* start */
    offset = 0;

    for(j = 0; j < TYPE_NUMARGS(i); ++j) {
      type = TYPE_ARG_TYPE(i, j);
      switch (type) {

        case (int)FIELD_INT:
        case (int)FIELD_TYPE:
          size = sizeof(meld_int);
          break;

        case (int)FIELD_FLOAT:
          size = sizeof(meld_float);
          break;

        case (int)FIELD_BOOL:
          size = sizeof(byte);
          break;

        case (int)FIELD_ADDR:
          size = sizeof(NodeID);
          break;

        case (int)FIELD_LIST_INT:
        case (int)FIELD_LIST_FLOAT:
        case (int)FIELD_LIST_ADDR:
        case (int)FIELD_SET_INT:
        case (int)FIELD_SET_FLOAT:
        case (int)FIELD_STRING:
          size = sizeof(void*);
          break;

        default:
          myassert(0);
          size = 0;
          break;
      }

      start[0] = size; /* argument size */
      start[1] = offset; /* argument offset */

      offset += size;
      start += 2;
    }
    arguments[i*2+1] = offset + TYPE_FIELD_SIZE; /* tuple size */
  }
}

/* Get TYPE id for useful types */
void init_consts() {
  tuple_type i;
  for (i = 0; i < NUM_TYPES; i++) {
    if (strcmp(TYPE_NAME(i), "edge") == 0)
      TYPE_EDGE = i;
    else if(strcmp(TYPE_NAME(i), "terminate") == 0)
      TYPE_TERMINATE = i;
  }
}

/* Saves the ID of useful types */
void init_all_consts(void) {
  // init_consts();
  tuple_type i;
  for (i = 0; i < NUM_TYPES; i++) {
    if (strcmp(TYPE_NAME(i), "tap") == 0)
      TYPE_TAP = i;
    else if (strcmp(TYPE_NAME(i), "neighbor") == 0)
      TYPE_NEIGHBOR = i;
    else if ( (strcmp(TYPE_NAME(i), "neighborCount" ) == 0) ||
        (strcmp(TYPE_NAME(i), "neighborcount" ) == 0) )
      TYPE_NEIGHBORCOUNT = i;
    else if (strcmp(TYPE_NAME(i), "vacant") == 0)
      TYPE_VACANT = i;
    else if (strcmp(TYPE_NAME(i), "setcolor") == 0)
      TYPE_SETCOLOR = i;
    else if (strcmp(TYPE_NAME(i), "setcolor2") == 0 )
      TYPE_SETCOLOR2 = i;
    else if (strcmp(TYPE_NAME(i), "position") == 0 )
      TYPE_POSITION = i;
    else if (strcmp(TYPE_NAME(i), "setposition") == 0)
      TYPE_SETPOSITION = i;
    else if (strcmp(TYPE_NAME(i), "ready") == 0)
      TYPE_READY = i;
    else if (strcmp(TYPE_NAME(i), "moveto") == 0)
      TYPE_MOVETO = i;
    else if (strcmp(TYPE_NAME(i), "readycount") == 0)
      TYPE_READYCOUNT = i;
    else if (strcmp(TYPE_NAME(i), "edge") == 0)
      TYPE_EDGE = i;
    else if (strcmp(TYPE_NAME(i), "unbroadcasted") == 0)
      TYPE_UNBROADCASTED = i;
  }
}

/* VM initialization routine */
void vm_init(void) {
//  srand(time(NULL));
//  time ( &starttime );
  // create_EventsList();
  init_all_consts();
  init_fields();
}
struct eventsList* malloc_eventsList() {
  struct eventsList* ret = &eventsListPool[eventsListPoolSize];
  eventsListPoolSize++;
  if(eventsListPoolSize == MAX_EVENTLISTPOOL) eventsListPoolSize = 0;
  return ret;
}
void enqueue_EventsList(Time time, int type){
  asm("NOP");//("");
  struct eventsList *tmp,*p;
  count_Events++;
  //tmp = malloc_eventsList();
  tmp=(struct eventsList *)malloc(sizeof(struct eventsList));
  if(tmp==NULL){
    asm("NOP");//("Memory not available\n");
    return;
  }
  tmp->eventsTime = time;
  tmp->eventsType = type;
  asm("NOP");//("");
  /*Queue is empty or item to be added has priority more than first element*/
  if( empty_EventsList() || time < front->eventsTime ){
    tmp->link=front;
    front=tmp;
  }
  else{
    p = front;
    while( p->link != NULL && p->link->eventsTime <= time )
      p=p->link;
    tmp->link=p->link;
    p->link=tmp;
  }
  asm("NOP");//("");
}

int dequeue_EventsList(){
  if( empty_EventsList() ){
    return 0;
  }
  else{
    front=front->link;
    count_Events--;
    return 1;
  }
}

struct eventsList frontelement_EventsList(){
  struct eventsList tmp;
  tmp.eventsTime=0;
  tmp.eventsType=0;
  tmp.link=NULL;
  if( empty_EventsList() ){
    asm("NOP");//("Queue Underflow\n");
  }
  else{
    tmp.eventsTime=front->eventsTime;
    tmp.eventsType=front->eventsType;
    tmp.link=front->link;
  }
  return tmp;
}

int empty_EventsList(){
  if(front == NULL)
    return 1;
  else
    return 0;
}

void display_EventsList()
{
  struct eventsList *ptr;
  ptr=front;
  if( empty_EventsList() )
    asm("NOP");//("Queue is empty\n");
  else{
    while(ptr!=NULL){
      asm("NOP");//("(%u, %d) ",ptr->eventsTime,ptr->eventsType);
      ptr=ptr->link;
    }
  }
  asm("NOP");//("\n");
}

int sizeof_EventsList(){
  return count_Events;
}

void enqueue_localEventsList(Time time, int type){
  struct localEventsList *tmp,*p;
  count_localEvents++;
  tmp=(struct localEventsList *)malloc(sizeof(struct localEventsList));
  if(tmp==NULL){
    asm("NOP");//("Memory not available\n");
    return;
  }

  tmp->eventsTime = time;
  tmp->eventsType = type;

  /*Queue is empty or item to be added has priority more than first element*/
  if( empty_localEventsList() || time < localfront->eventsTime ){
    tmp->link=localfront;
    localfront=tmp;
  }
  else{
    p = localfront;
    while( p->link != NULL && p->link->eventsTime <= time )
      p=p->link;
    tmp->link=p->link;
    p->link=tmp;
  }
}

int dequeue_localEventsList(){
  struct localEventsList *tmp;
  // int item;
  if( empty_localEventsList() ){
    asm("NOP");//("Queue Underflow\n");
    // exit(1);
    return -1;
  }
  else{
    tmp=localfront;
    // item=tmp->info;
    localfront=localfront->link;
    free(tmp);
    count_localEvents--;
  }
  return 1;
}

struct localEventsList frontelement_localEventsList(){
  struct localEventsList tmp;
  tmp.eventsTime=0;
  tmp.eventsType=0;
  tmp.link=NULL;
  if( empty_localEventsList() ){
    asm("NOP");//("Queue Underflow\n");
  }
  else{
    tmp.eventsTime=localfront->eventsTime;
    tmp.eventsType=localfront->eventsType;
    tmp.link=localfront->link;
  }
  return tmp;
}

bool empty_localEventsList(){
  if(localfront == NULL)
    return true;
  else
    return false;
}

void display_localEventsList()
{
  struct localEventsList *ptr;
  ptr=localfront;
  if( empty_localEventsList() )
    asm("NOP");//("Queue is empty\n");
  else{
    while(ptr!=NULL){
      asm("NOP");//("(%u, %d) ",ptr->eventsTime,ptr->eventsType);
      ptr=ptr->link;
    }
  }
  asm("NOP");//("\n");
}

int sizeof_localEventsList(){
  return count_localEvents;
}

void schedule(Time time, int type){
  //add the event into the map
  asm("NOP");//("Inserting into the map: %d\n", type);
  enqueue_EventsList(time, type);
  asm("NOP");//("");
}

void scheduleLocalEvent(Time time, int type){
  asm("NOP");//("");
  enqueue_localEventsList(time,type);
  asm("NOP");//("");
  if(sizeof_localEventsList()==1){
    schedule(time,EVENT_PROCESS_LOCAL_EVENT);
  }
}

void startup(){
  static int x,y,z;
  x=(int) (GetCoordinate().x*1000);//(int)(GetCoordinate().x*100);//int x = getPosX();
  y=(int) (GetCoordinate().y*1000);//(int)(GetCoordinate().y*100);// = getPosY();
  z=0;// = getPosZ(); CONTRACT
  asm("NOP");
  enqueue_position((meld_int)x,(meld_int)y,(meld_int)z);
  asm("NOP");//("after enqueue_position\n");
  schedule(getTime(),EVENT_COMPUTE_PREDICATE);
}

void processLocalEvent(Time time, int type) {

  switch (type) {
    case EVENT_COMPUTE_PREDICATE:
      {
        asm("NOP");//("local event EVENT_COMPUTE_PREDICATE\n");
        //Call the VM function to process one rule
        processOneRule();
        //Add another compute event on condition
        //if...
        if(isWaiting()){
          // random delay before recomputing a predicate
          // between 0.1 ms and 1ms
          int delay = (getNextRandomNumber() % (1000 - 100 +1 )) + 100;
          schedule(getTime()+delay, EVENT_COMPUTE_PREDICATE);
          // schedule(new ComputePredicateEvent(now()+delay, bb));
        }
      }
      break;
    case EVENT_STOP:
      {
        asm("NOP");//("local event EVENT_STOP\n");
      }
      break;
    case EVENT_ADD_NEIGHBOR:
      {
        // asm("NOP");//("EVENT_ADD_NEIGHBOR\n");
      }
      break;
    case EVENT_ADD_EDGE:
      {
        asm("NOP");//("local event EVENT_ADD_EDGE\n");
         //GET TARGET ID FROM Event Pointer!! = (boost::static_pointer_cast<AddEdgeEvent>(pev))->target;
        //enqueue_edge(target);
        //schedule(getTime(), EVENT_COMPUTE_PREDICATE); //new ComputePredicateEvent(BaseSimulator::getScheduler()->now(), bb));
      }
    case EVENT_REMOVE_NEIGHBOR:
      {
        asm("NOP");//("local event EVENT_REMOVE_NEIGHBOR\n");
        unsigned int face; //GET FACE FROM Event Pointer!! = (boost::static_pointer_cast<AddNeighborEvent>(pev))->face;
        neighbors[face] = 0;
        enqueue_face(neighbors[face], face, -1);
        schedule(getTime(), EVENT_COMPUTE_PREDICATE);
      }
      break;
    case EVENT_SET_POSITION:
      {
        asm("NOP");//("local event EVENT_SET_POSITION\n");
        // getTime();
        // int x;  //GET x from Event Pointer!! = (boost::static_pointer_cast<SetPositionEvent>(pev))->x;
        // int y;  //GET y from Event Pointer!! = (boost::static_pointer_cast<SetPositionEvent>(pev))->y;
        // int z;  //GET z from Event Pointer!! = (boost::static_pointer_cast<SetPositionEvent>(pev))->z;
        // bb->setPosition(x,y,z);
        // vm->enqueue_position((meld_int)bb->position.pt[0],(meld_int)bb->position.pt[1],(meld_int)bb->position.pt[2]);
        // BaseSimulator::getScheduler()->schedule(new ComputePredicateEvent(BaseSimulator::getScheduler()->now(), bb));
        // info << "set position " << x << " " << y << " " << z;
      }
      break;
    case EVENT_MOVE_TO:
      {
        asm("NOP");//("local event EVENT_MOVE_TO\n");
        //typeCoordinate location;
        //location=GetCoordinate();
        static float x, y;
        
        x = targetPos[0] / 1000.0;
        y = targetPos[1] / 1000.0;
        asm("NOP");
        //rotateTo(x,y,ANGLESPEED,0);
        ControlRobotgo2Position(x,y,40);
        haltornot = 0;
        asm("NOP");
        //Set the global position to final position and enqueue it.
        enqueue_position((meld_int)targetPos[0], (meld_int)targetPos[1], (meld_int)targetPos[2]);
        schedule(getTime(), EVENT_COMPUTE_PREDICATE);
        asm("NOP");
       }
       break;
//       /*The interface being connected is tested in function tuple_send of the MeldInterpVM*/
//     case EVENT_SEND_MESSAGE:
//       {
//         MessagePtr message = (boost::static_pointer_cast<VMSendMessageEvent>(pev))->message;
//         P2PNetworkInterface *interface = (boost::static_pointer_cast<VMSendMessageEvent>(pev))->sourceInterface;
//         BlinkyBlocks::getScheduler()->schedule(new NetworkInterfaceEnqueueOutgoingEvent(BaseSimulator::getScheduler()->now(), message, interface));
//         //info << "sends a message at face " << NeighborDirection::getString(bb->getDirection(interface))  << " to " << interface->connectedInterface->hostBlock->blockId;
//       }
//       break;
    case EVENT_SEND_MESSAGE_TO_BLOCK:
      {
        asm("NOP");//("local event EVENT_SEND_MESSAGE_TO_BLOCK\n");
        //Broadcast ready Message!
        boardCastInfos();
        //SetLeftWheelGivenSpeed(0);
        //SetRightWheelGivenSpeed(0); 
        //vTaskDelay(300);
//        while(1){
//          int id;
//          for(id=1;id<=ROBOTS;id++){
//            if(id!=rid && !isReady(id)){
//              break;
//            }
//          }
//          if(id==ROBOTS+1){
//            break;
//          }
//          //halt(1);
//          asm("NOP");
//        }
        //while(1){
          //if(getTime() > 15000){
            for(u8 i=1;i<=ROBOTS;++i){
              if(i!=rid){
                enqueue_ready(i);
              }
            }
            //break;
          //}
        //}
      }
      break;
    case EVENT_RECEIVE_MESSAGE_FROM_BLOCK: /*EVENT_NI_RECEIVE: */
      {
        asm("NOP");//("local event EVENT_RECEIVE_MESSAGE_FROM_BLOCK\n");
        // asm("NOP");//("waah!\n");
      }
      break;
    case EVENT_ACCEL:
      {
      }
      break;
    case EVENT_SHAKE:
      {
      }
      break;
    case EVENT_SET_DETERMINISTIC:
      {
      }
      break;
    case EVENT_END_POLL:
      {
        asm("NOP");//("local event EVENT_END_POLL\n");
        // polling = false;
        /*Not written yet
          Need to check what this is for*/
        // info << "Polling time period ended";
      }
      break;
    case EVENT_ADD_TUPLE:
    {
      asm("NOP");//("local event EVENT_ADD_TUPLE\n");
      // receive_tuple(1, tuple, face);
      schedule(getTime(), EVENT_COMPUTE_PREDICATE);
      // schedule(new ComputePredicateEvent(BaseSimulator::getScheduler()->now(), bb));
      //info << "Adding tuple";
    }
      break;
    case EVENT_REMOVE_TUPLE:
    {
      asm("NOP");//("local event EVENT_REMOVE_TUPLE\n");
    //   this->vm->receive_tuple(-1, boost::static_pointer_cast<RemoveTupleEvent>(pev)->tuple, boost::static_pointer_cast<RemoveTupleEvent>(pev)->face);
    //   BaseSimulator::getScheduler()->schedule(new ComputePredicateEvent(BaseSimulator::getScheduler()->now(), bb));
      //info << "Removing tuple";
    }
      break;
    default:
      asm("NOP");//("*** ERROR *** : unknown local event\n");
      break;
  }
}

void consume_BlockEvent(Time time, int type){
  //return;
  switch (type) {
    case EVENT_CODE_START:
      {
        asm("NOP");//("consume EVENT_CODE_START\n");
        startup();
      }
      break;
    case EVENT_PROCESS_LOCAL_EVENT:
      {
        asm("NOP");//("consume EVENT_PROCESS_LOCAL_EVENT\n");
        struct localEventsList temp;
        temp = frontelement_localEventsList();
        if(dequeue_localEventsList()==1){
          processLocalEvent(temp.eventsTime,temp.eventsType);
        }

        if(sizeof_localEventsList()>0){
          schedule(getTime(),EVENT_PROCESS_LOCAL_EVENT);
          // asm("NOP");//("scheduled2\n");
        }
      }
      break;
    case EVENT_RECEIVE_MESSAGE_FROM_BLOCK:
      {
        asm("NOP");//("consume EVENT_RECEIVE_MESSAGE_FROM_BLOCK\n");
        scheduleLocalEvent(getTime(),EVENT_RECEIVE_MESSAGE_FROM_BLOCK);
      }
      break;
    case EVENT_COMPUTE_PREDICATE:
      {
        asm("NOP");//("consume EVENT_COMPUTE_PREDICATE\n");
        scheduleLocalEvent(getTime(),EVENT_COMPUTE_PREDICATE);
      }
      break;
    case EVENT_STOP:
      {
        asm("NOP");//("consume EVENT_STOP\n");
        state = ENDED;
      }
      break;
    case EVENT_ADD_NEIGHBOR:
      {
        asm("NOP");//("consume EVENT_ADD_NEIGHBOR\n");
        scheduleLocalEvent(getTime(),EVENT_ADD_NEIGHBOR);
      }
      break;
    case EVENT_ADD_EDGE:
      {
        asm("NOP");//("consume EVENT_ADD_EDGE\n");
        scheduleLocalEvent(getTime(),EVENT_ADD_EDGE);
      }
      break;
    case EVENT_REMOVE_NEIGHBOR:
      {
        asm("NOP");//("consume EVENT_REMOVE_NEIGHBOR\n");
        scheduleLocalEvent(getTime(),EVENT_REMOVE_NEIGHBOR);
      }
      break;
    case EVENT_SET_POSITION:
      {
        asm("NOP");//("consume EVENT_SET_POSITION\n");
        scheduleLocalEvent(getTime(),EVENT_SET_POSITION);
      }
      break;
    case EVENT_MOVE_TO:
      {
        asm("NOP");//("consume EVENT_MOVE_TO\n");
        scheduleLocalEvent(getTime(),EVENT_MOVE_TO);
      }
       break;
    case EVENT_SEND_MESSAGE_TO_BLOCK:
      {
        asm("NOP");//("consume EVENT_SEND_MESSAGE_TO_BLOCK\n");
        scheduleLocalEvent(getTime(),EVENT_SEND_MESSAGE_TO_BLOCK);
      }
      break;
    case EVENT_END_POLL:
      {
        asm("NOP");//("consume EVENT_END_POLL\n");
        scheduleLocalEvent(getTime(),EVENT_END_POLL);
      }
      break;
    case EVENT_ADD_TUPLE:
      {
        asm("NOP");//("consume EVENT_ADD_TUPLE\n");
        scheduleLocalEvent(getTime(),EVENT_ADD_TUPLE);
      }
      break;
    case EVENT_REMOVE_TUPLE:
      {
        asm("NOP");//("consume EVENT_REMOVE_TUPLE\n");
        scheduleLocalEvent(getTime(),EVENT_REMOVE_TUPLE);
      }
      break;
    default:
      // ERRPUT << "*** ERROR *** : unknown local event";
      asm("NOP");//("*** ERROR *** : unknown local event\n");
      break;
  }
  asm("NOP");
}

void runStraight(float x, float y, float speed) {
  typeCoordinate start = GetCoordinate();
  if(GoalInFront(x,y) == true)
	return;
  //sleepSafe(FRONT_SIGHT_LENGTH);  
  typeCoordinate nowp;
  int tempTime = 500;
  int  nearTime = 0;
  while (1) {	
//	for(i = 0;i < 1; i++){
//	while(1){
//	  if(sleepSafe(FRONT_SIGHT_LENGTH) == 0)
//		break;
//	}
	  SetLeftWheelGivenSpeed(speed);
	  SetRightWheelGivenSpeed(speed);
	  vTaskDelay(tempTime - nearTime);
	  if(GoalInFront(x,y) == true)
		return;
//	}
	nowp =  GetCoordinate();
//	if(nowp.x >= x)
//	  return;
	float dist = getDistance2(nowp.x, nowp.y, x, y);
	if(dist >= 0.06)
	{
      rotateTo(x,y,ANGLESPEED,0);	
	}
	else {
	  rotateTo(x,y,ANGLESPEED,1);	
//	  nearTime = 200;
	}
 }  /* end of while */
} /*end of ControlRobotgo2Position*/


void multiRobotVMTask (void *pvParameters){
  
  halt(5);
  typeCoordinate nowp = GetCoordinate();
  static float x,y;
  x = nowp.x;
  y = 0.38;
  rotateTo(x,y,5,1);
  runStraight(x,y,40);
  boardCastInfos();
  while(1){
    int id;
    for(id=1;id<=ROBOTS;id++){
      if(id!=rid && !isReady(id)){
        break;
      }
    }
    if(id==ROBOTS+1){
      break;
    }
    //halt(1);
    asm("NOP");
  }
  nowp = GetCoordinate();
  ControlRobotgo2Position(nowp.x+1000,nowp.y,40);
  halt(10);
  
  
  
  
  
  
  
//  readProgram();
//  TYPE_EDGE = -1;
//  TYPE_TERMINATE = -1;
//  TYPE_NEIGHBORCOUNT = -1;
//  TYPE_NEIGHBOR = -1;
//  TYPE_VACANT = -1;
//  TYPE_TAP = -1;
//  TYPE_SETCOLOR = -1;
//  TYPE_SETCOLOR2 = -1;
//  TYPE_POSITION = -1;
//  TYPE_SETPOSITION = -1;
//  TYPE_MOVETO = -1;
//  TYPE_READY = -1;
//  
//  vm_alloc();
//  vm_init();
//  hasWork = true;
//  polling = false;
//  deterministicSet = false;
//  firstStart = true;
//  currentLocalDate = 0;
//  
//  enqueue_init();
//
//  numNeighbors = getNeighborCount();
//  enqueue_count(numNeighbors, 1);
//
//  schedule(getTime(),EVENT_COMPUTE_PREDICATE);
//  schedule(getTime(),EVENT_CODE_START);
//  int i;
//  for(i=1;i<=ROBOTS;i++){
//    enqueue_edge(i);
//  }
//  schedule(getTime(), EVENT_COMPUTE_PREDICATE);
//  struct eventsList first;
//  
////  static portTickType time;
//  startTime = xTaskGetTickCount();
//  asm("NOP");
//  while(1){
//    //SetLeftWheelGivenSpeed(20);
//    //SetRightWheelGivenSpeed(20);
//    //vTaskDelay(2500);
//    //halt(1);
//      //if(empty_EventsList()){
//        //break;
//      //}
//    //time = xTaskGetTickCount();
//    //asm("NOP");
//      first = frontelement_EventsList();
//      asm("NOP");
//      //currentDate = first.eventsTime;
//      dequeue_EventsList();
//      consume_BlockEvent(first.eventsTime, first.eventsType);
//        
//  }
}