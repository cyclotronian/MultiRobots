#include "halUsart.h"


void halUsart1Init(void){
  //1. ����IO��
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);    //USART1->�˿���ӳ��
  //2. ����USART1
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
  //3. USART1_RX�ж�����
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;           //ռ��ʽ
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                  //��Ӧʽ
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);                                     //���ṹ�嶪�����ú�������д�뵽��Ӧ�Ĵ�����
  USART_ClearFlag(USART1, USART_FLAG_CTS | USART_FLAG_LBD  |  USART_FLAG_TC  | USART_FLAG_RXNE );
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                      //���ڽ��ջ�������Ϊ���ж�
}

void halUsart1TxByte(u8 data){
  while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
  USART_SendData(USART1, data);
}

void halUsart1TxPacket(u8* data, u8 len){
  unsigned char i;
  for (i=0;i<len;i++) {
    halUsart1TxByte(*(data+i));
  }
}


