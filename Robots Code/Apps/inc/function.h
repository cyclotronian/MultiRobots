#ifndef __FUNCTION_H
#define __FUNCTION_H


#include "string.h"
#include "math.h"
#include "stm32f10x.h"
#include "stdbool.h"


extern float GetMaxData(float* data, u16 len);
extern float GetMinData(float* data, u16 len);
extern float GetMaxMinDiffData(float* data, u16 len);
extern float GetAverageData(float* data, u16 len);
extern void SortDataMax2Min(float* dataI, float* dataO, u16 len);
extern void SortDataMin2Max(float* dataI, float* dataO, u16 len);
extern float FloatAbs(float data);


#endif

