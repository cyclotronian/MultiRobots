#include "halADC.h"


////////////////////////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////
void halAdcGpioInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void halAdcInit(unsigned char ch) {
  ADC_InitTypeDef ADC_InitStructure;                                   //定义ADC初始化结构体变量
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                   //ADC1和ADC2工作在独立模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;                         //使能扫描
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                   //ADC转换工作在连续模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //有软件控制转换
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               //转换数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 1;                              //转换通道为通道7
  ADC_Init(ADC1, &ADC_InitStructure);                                  //初始化ADC
  ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_28Cycles5);
  //ADC1选择信道14,音序器等级1,采样时间239.5个周期
  ADC_DMACmd(ADC1, DISABLE);                                           //使能ADC1模块DMA
  ADC_Cmd(ADC1, ENABLE);                                               //使能ADC1
  ADC_ResetCalibration(ADC1);                                          //重置ADC1校准寄存器
  while(ADC_GetResetCalibrationStatus(ADC1));                          //等待ADC1校准重置完成
  ADC_StartCalibration(ADC1);                                          //开始ADC1校准
  while(ADC_GetCalibrationStatus(ADC1));                               //等待ADC1校准完成
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);                              //使能ADC1软件开始转换x
}