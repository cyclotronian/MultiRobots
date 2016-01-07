#ifndef __VMELDTASK_H
#define __VMELDTASK_H

#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "vTasks.h"
#include "apps.h"
#include "stdbool.h"

#include "vInfoList.h"

#define M_PI  3.14159265358979323846f
#define SLOW  100.0f
#define STOP   40.0f
struct fact
{
	int code;
	int robot_id;
	double x;
	double y;
};

struct database{
	struct fact base;
	struct database *next;
};
extern void vMeldTask( void *pvParameters );
void actuate_move(struct database **, struct fact);
void actuate_rotate(struct fact);
void add_to_database(struct database **, struct fact);
struct fact search_in_database1(struct database *, int, int);
int search_in_database(struct database *, struct fact);
void delete_from_database(struct database **, struct fact);

#endif

