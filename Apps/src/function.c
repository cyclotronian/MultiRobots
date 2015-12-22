#include "function.h"

float GetMaxData(float* data, u16 len) {
  float maxData;
  u16 i;
  maxData = *data;
  for (i=0;i<len;i++) {
    if (maxData < *(data+i)) {
      maxData = *(data+i);
    }
  }
  return maxData;
}

float GetMinData(float* data, u16 len) {
  float minData;
  u16 i;
  minData = *data;
  for (i=0;i<len;i++) {
    if (minData > *(data+i)) {
      minData = *(data+i);
    }
  }
  return minData;
}

float GetMaxMinDiffData(float* data, u16 len){
  return (GetMaxData(data,len)-GetMinData(data, len));
}

float GetAverageData(float* data, u16 len){
  float sum=0;
  u16 i;
  for (i=0;i<len;i++) {
    sum+=*(data+i);
  }
  return (sum/len);
}

void SortDataMax2Min(float* dataI, float* dataO, u16 len) {
  u16 i, j;
  float midDdata;
  for (i=0;i<len;i++) {
    for (j=i;j<len;j++) {
      if (*(dataI+i) <*(dataI+j)){
        midDdata   = *(dataI+i);
        *(dataI+i) = *(dataI+j);
        *(dataI+j) = midDdata;
      }
    }
    *(dataO+i) = *(dataI+i);
  }
}

void SortDataMin2Max(float* dataI, float* dataO, u16 len) {
  u16 i, j;
  float midDdata;
  for (i=0;i<len;i++) {
    for (j=i;j<len;j++) {
      if (*(dataI+i) > *(dataI+j)){
        midDdata   = *(dataI+i);
        *(dataI+i) = *(dataI+j);
        *(dataI+j) = midDdata;
      }
    }
    *(dataO+i) = *(dataI+i);
  }
}

float FloatAbs(float data) {
  if (data<0.0f) {
    return (0.0f-data);
  }
  return data;
}



