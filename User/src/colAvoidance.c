
#include <math.h>
#include "colAvoidance.h"
#include "vDemoTask.h"


float cross(float x1, float y1, float x2, float y2) {
	return x1*y2 - y1*x2;
}

float mysin(float a) {
	return sin(a*PI/180);
}

float mycos(float a) {
	return cos(a*PI/180);
}

void getintersection(float x1, float y1, float ang1, float x2, float y2, float ang2, float *x, float *y) {
	float vx = mysin(ang1), vy = mycos(ang1);
	float wx = mysin(ang2), wy = mycos(ang2);
	float ux = x1 - x2, uy = y1 - y2;
	float t = cross(wx, wy, ux, uy) / cross(vx, vy, wx, wy);
	*x = x1 + vx * t;
	*y = y1 = vy * t;
	//printf("%lf %lf\n", *x, *y);
}

int intersected(float x1, float y1, float ang1, float x2, float y2, float ang2) {
	float x, y;
	getintersection(x1, y1, ang1, x2, y2, ang2, &x, &y);
	float vx = x - x1, vy = y - y1;
	return mycos(ang1)*vx + mysin(ang1)*vy > -1e-3;
}