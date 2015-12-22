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
  ADC_InitTypeDef ADC_InitStructure;                                   //����ADC��ʼ���ṹ�����
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                   //ADC1��ADC2�����ڶ���ģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;                         //ʹ��ɨ��
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                   //ADCת������������ģʽ
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //���������ת��
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               //ת�������Ҷ���
  ADC_InitStructure.ADC_NbrOfChannel = 1;                              //ת��ͨ��Ϊͨ��7
  ADC_Init(ADC1, &ADC_InitStructure);                                  //��ʼ��ADC
  ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_28Cycles5);
  //ADC1ѡ���ŵ�14,�������ȼ�1,����ʱ��239.5������
  ADC_DMACmd(ADC1, DISABLE);                                           //ʹ��ADC1ģ��DMA
  ADC_Cmd(ADC1, ENABLE);                                               //ʹ��ADC1
  ADC_ResetCalibration(ADC1);                                          //����ADC1У׼�Ĵ���
  while(ADC_GetResetCalibrationStatus(ADC1));                          //�ȴ�ADC1У׼�������
  ADC_StartCalibration(ADC1);                                          //��ʼADC1У׼
  while(ADC_GetCalibrationStatus(ADC1));                               //�ȴ�ADC1У׼���
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);                              //ʹ��ADC1�����ʼת��x
}