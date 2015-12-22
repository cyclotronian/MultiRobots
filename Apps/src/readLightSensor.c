#include "readLightSensor.h"

void LightSensorInit(void) {
  halAdcGpioInit();
  halAdcInit(ADC_Channel_7);
}

float ReadLightSensor(void){
  float lightSensorTemp;
  u32 lightSensorHexTemp=0;
  u8  i;
  for (i=0;i<LIGHT_SENSOR_READ_TIMES;i++)  {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    lightSensorHexTemp += ADC_GetConversionValue(ADC1);
  }
  lightSensorTemp = (float)(lightSensorHexTemp)/LIGHT_SENSOR_READ_TIMES;
  return lightSensorTemp;
}

