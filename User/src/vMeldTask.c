#include "vMeldTask.h"
#include "robot.h"
#include "vInfoList.h"
#include <math.h>
/*
#define LINESPEED 20
#define ANGLESPEED 5

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
*/
void RemoveSpaces(char* source)
{
  char* i = source;
  char* j = source;
  while(*j != 0)
  {
    *i = *j++;
    if(*i != ' ')
      i++;
  }
  *i = 0;
}

int inRange(double x1, double y1, double x2, double y2){
	int retval=0;
	double distance = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
	if(distance<50){
		retval=1;
	}
	return retval;
}
//-------------END OF HELPER FUNCTIONS-------------------------


//-------------DATABASE AND ACTUATION FUNCTION PROTOTYPES------




#define POSITION		1
#define MOVE			2
#define ROTATE			3
#define RULE			10
#define FACT			11

int type(char *statement){
	int i=0,retval=FACT;
	for(i=0;statement[i]!='.';i++){
		if((statement[i]=='-')&&(statement[i+1]=='o')){
			retval=RULE;
			break;
		}
	}
	return retval;
}

struct fact getFact(char *primitive){
	struct fact retFact;
	int i,j;
	char *param;
	char bot_id[10]={'\0'};
	char x_pos[10]={'\0'};
	char y_pos[10]={'\0'};

	if(primitive[0]=='p'){
		retFact.code=POSITION;
	}
	if(primitive[0]=='m'){
		retFact.code=MOVE;
	}
	
	param=strchr(primitive,'(');
	i=1; j=0;
	while(param[i]!=','){
		bot_id[j]=param[i];
		i++; j++;
	}
	i++; j=0;
	while(param[i]!=','){
		x_pos[j]=param[i];
		i++; j++;
	}
	i++; j=0;
	while(param[i]!=')'){
		y_pos[j]=param[i];
		i++; j++;
	}

	retFact.robot_id = atoi(bot_id);
	retFact.x = atof(x_pos);
	retFact.y = atof(y_pos);

	return retFact;
}

void actuate_move(struct database** head_ref, struct fact F){
	struct fact newFact,oldFact;
	oldFact=search_in_database1((*head_ref), POSITION,F.robot_id);
	delete_from_database(head_ref, oldFact);
	//----movement actuation in robots-------------

	//ControlRobotgo2Position(F.x, F.y, 20/*float speed*/);

	//---------------------------------------------
	newFact.code=POSITION;
	newFact.robot_id=F.robot_id;
	newFact.x=F.x;
	newFact.y=F.y;
	//printf("Moving robot %d from (%lf, %lf) to (%lf, %lf)\n", newFact.robot_id, oldFact.x, oldFact.y, newFact.x, newFact.y);
	add_to_database(head_ref, newFact);
}

void add_to_database(struct database** head_ref, struct fact F){
	//printf("Adding fact - code: %d, robot_id: %d, x: %lf, y:%lf to the DATABASE\n", F.code, F.robot_id, F.x, F.y);
	if(F.code==MOVE){
		actuate_move(head_ref, F);
	}
	// if(F.code==ROTATE){
	// 	actuate_rotate(F);
	// }
	if(F.code==POSITION){
		struct database* newNode = (struct database*) malloc(sizeof(struct database));
		newNode->base.code=F.code;
		newNode->base.robot_id=F.robot_id;
		newNode->base.x=F.x;
		newNode->base.y=F.y;
		newNode->next=(*head_ref);
		(*head_ref)=newNode;
	}
}

struct fact search_in_database1(struct database *head, int code, int robot_id){
	//printf("Searching fact - code: %d, robot_id: %d in the DATABASE\n", code, robot_id);
	struct fact newFact;
	newFact.code=code;
	newFact.robot_id=robot_id;
	struct database* current = head;
	while (current != NULL) {
		if((current->base.code==code) && (current->base.robot_id==robot_id)){
			newFact.x=current->base.x;
			newFact.y=current->base.y;
			break;
		}
		current = current->next;
	}
	return newFact;
}

int search_in_database(struct database *head, struct fact F){
	//printf("Searching fact - code: %d, robot_id: %d, x: %lf, y:%lf in the DATABASE\n", F.code, F.robot_id, F.x, F.y);
	int retval=0;
	struct database* current = head;
	while (current != NULL) {
		if((current->base.code==F.code) && (current->base.robot_id==F.robot_id) && inRange(current->base.x,current->base.y,F.x,F.y)){
			retval=1;
			//printf("Fact Found in Database!\n");
			break;
		}
		current = current->next;
	}
	return retval;
}

void delete_from_database(struct database** head_ref, struct fact F){
	//printf("Deleting fact - code: %d, robot_id: %d, x: %lf, y:%lf from the DATABASE\n", F.code, F.robot_id, F.x, F.y);
	struct database* temp = *head_ref, *prev;
	if(temp!=NULL && (temp->base.code==F.code) && (temp->base.robot_id==F.robot_id) && inRange(temp->base.x,temp->base.y,F.x,F.y)){
		//printf("Node Found in the head of the database!\n");
		*head_ref=temp->next;
		free(temp);
		return;
	}
	while(temp!=NULL && ((temp->base.code!=F.code) || (temp->base.robot_id!=F.robot_id) || !inRange(temp->base.x,temp->base.y,F.x,F.y))){
		prev=temp;
		temp=temp->next;
	}

	if(temp==NULL) return;
	prev->next=temp->next;

	free(temp);
	//printf("Node deleted!\n");
}

// void print_database(struct database *head){
// 	if(head->next==NULL)
// 		//printf("(%d, %d, %lf, %lf)", head->base.code, head->base.robot_id, head->base.x, head->base.y);
// 	else{
// 		struct database* current = head;
// 		while (current->next != NULL) {
// 			//printf("(%d, %d, %lf, %lf)->", current->base.code, current->base.robot_id, current->base.x, current->base.y);
// 			current = current->next;
// 		}
// 		//printf("(%d, %d, %lf, %lf)", current->base.code, current->base.robot_id, current->base.x, current->base.y);
// 	}
// 	//printf("\n");
// }


// void actuate_rotate(struct fact F){
// 	//printf("Rotating by an angle\n");
// }

void vMeldTask( void *pvParameters ) {
  halt(1);
  //-------CREATE DATABASE-----------------------------
	struct database *databaseHead = NULL;
		

    //-------READ A LINE OF CODE-----------------------
		char line_of_code[100]="position(1,14.5,15.1)";
	 
	 	//-------REMOVE SPACES-------------------------------
	 	RemoveSpaces(line_of_code);

	 	//-------ADD TO DATABASE IF IT IS A RULE-------------
	 	if(type(line_of_code)==FACT){
	 		struct fact Fact;
	 		Fact = getFact(line_of_code);
	 		//printf("It's a FACT\n");
	 		add_to_database(&databaseHead,Fact);
	 		// print_database(databaseHead);
	 	}

	 	//------GET HEAD AND BODY IF IT IS A RULE------------
	 	if(type(line_of_code)==RULE){
	 	
	 		int i,state=0,headsize=0,bodysize=0;
	 		char head[50]={'\0'},body[50]={'\0'};

		 	for(i=0;line_of_code[i]!='\0';i++){
		 		
		 		if((line_of_code[i]=='-')&&(line_of_code[i+1]=='o')){
		 			state=1;
		 		}
		 		if(state==0){
		 			body[i]=line_of_code[i];
		 			bodysize++;
		 		}
		 		else{
		 			if(((line_of_code[i]=='-')&&(line_of_code[i+1]=='o')) || ((line_of_code[i-1]=='-')&&(line_of_code[i]=='o'))){
		 				continue;
		 			}
		 			head[i-bodysize-2]=line_of_code[i];
		 			headsize++;
		 		}
		 	}

		 	//printf("It's a RULE\nHead: %s\nBody: %s\n", head,body);
		 	struct fact Fact;
	 		Fact = getFact(body);
	 		// print_database(databaseHead);
	 		if(search_in_database(databaseHead, Fact)){
	 			Fact=getFact(head);
	 			add_to_database(&databaseHead,Fact);
	 			// print_database(databaseHead);
	 		}
		}


		strcpy(line_of_code,"position(1,14,15) -o move(1,100,150)");
	 
	 	//-------REMOVE SPACES-------------------------------
	 	RemoveSpaces(line_of_code);

	 	//-------ADD TO DATABASE IF IT IS A RULE-------------
	 	if(type(line_of_code)==FACT){
	 		struct fact Fact;
	 		Fact = getFact(line_of_code);
	 		//printf("It's a FACT\n");
	 		add_to_database(&databaseHead,Fact);
	 		// print_database(databaseHead);
	 	}

	 	//------GET HEAD AND BODY IF IT IS A RULE------------
	 	if(type(line_of_code)==RULE){
	 	
	 		int i,state=0,headsize=0,bodysize=0;
	 		char head[50]={'\0'},body[50]={'\0'};

		 	for(i=0;line_of_code[i]!='\0';i++){
		 		
		 		if((line_of_code[i]=='-')&&(line_of_code[i+1]=='o')){
		 			state=1;
		 		}
		 		if(state==0){
		 			body[i]=line_of_code[i];
		 			bodysize++;
		 		}
		 		else{
		 			if(((line_of_code[i]=='-')&&(line_of_code[i+1]=='o')) || ((line_of_code[i-1]=='-')&&(line_of_code[i]=='o'))){
		 				continue;
		 			}
		 			head[i-bodysize-2]=line_of_code[i];
		 			headsize++;
		 		}
		 	}

		 	//printf("It's a RULE\nHead: %s\nBody: %s\n", head,body);
		 	struct fact Fact;
	 		Fact = getFact(body);
	 		// print_database(databaseHead);
	 		if(search_in_database(databaseHead, Fact)){
	 			Fact=getFact(head);
	 			add_to_database(&databaseHead,Fact);
	 			// print_database(databaseHead);
	 		}
		}
	
}
