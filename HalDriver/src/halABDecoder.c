#include "halABDecoder.h"

void halABDecoderGPIOConfig(void) {
  //ABX/Y   -> PB8  Select the 1st or 2nd axis data to be read. Low bit enables the 1st axis data, while high bit enables the 2nd axis data.
  //ABSEL2  -> PB9
  //ABSEL1  -> PB12 These CMOS inputs directly controls which data byte from the position latch is enabled into the 8-bit tri-state output buffer
  //ABEN1   -> PB10
  //ABEN2   -> PB11 These CMOS control pins are set to high or low to activate the selected count mode before the decoding begins
  //ABOEN   -> PB13 This CMOS active low input enables the tri-state output buffers
  //ABRSTNX -> PB15
  //ABRSTNY -> PB14 This active low Schmitt-trigger input clears the internal position counter and the position latch. It also resets the inhibit logic.
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11 |
                                GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,  GPIO_Pin_14);
  GPIO_SetBits(GPIOB,  GPIO_Pin_15);
  //ABD0~ABD7 -> PC0~PC7    These LSTTL-compatible tri-state outputs form an 8-bit output ports
  //ABCNTDECY -> PC8
  //ABCNTDECX -> PC9        A pulse is presented on this LSTTL-compatible output when the quadrature decoder (4x/2x/1x) has detected a state transition
  //ABCNTCASX -> PC10
  //ABCNTCASY -> PC11       A pulse is presented on this LSTTL-compatible output when the HCTL-2032 / 2032-SC internal counter overflows or underflows
  //ABU/D\Y   -> PC12
  //ABU/D\X   -> PC13       This LSTTL-compatible output allows the user to determine whether the IC is counting up or down and is intended to be used with the CNTDEC and CNTCAS outputs
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2 | GPIO_Pin_3 |
                                GPIO_Pin_4  | GPIO_Pin_5  | GPIO_Pin_6 | GPIO_Pin_7 |
                                GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10| GPIO_Pin_11|
                                GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//ABEN1   -> PB10
//ABEN2   -> PB11
//EN1 EN2 => (0,0)->ILLEGAL (1,0)->4X (0,1)->2X  (1,1)->1X
void halABDecoderSelCountMode(u8 mode){
  switch(mode) {
    case COUNT_MODE_X1:  {
      GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
      break;
    }
    case COUNT_MODE_X2:  {
      GPIO_SetBits(GPIOB,  GPIO_Pin_11);
      GPIO_ResetBits(GPIOB,  GPIO_Pin_10);
      break;
    }
    case COUNT_MODE_X4:  {
      GPIO_SetBits(GPIOB,  GPIO_Pin_10);
      GPIO_ResetBits(GPIOB,  GPIO_Pin_11);
      break;
    }
    default: {
      break;
    }
  }
}

void halABDecoderInit(void) {
  halABDecoderGPIOConfig();
  halABDecoderSelCountMode(COUNT_MODE_X4);
  
}

//ABOEN     -> PB13
//ABSEL2    -> PB9
//ABSEL1    -> PB12
//ABD0~ABD7 -> PC0~PC7
////////////////////////////////////////////////////////////////////////////////
//右轮：编码方向正常
////////////////////////////////////////////////////////////////////////////////
signed long halABXDecoderReadData(void) {
  signed long abData=0;
  GPIO_ResetBits(GPIOB,  GPIO_Pin_8);      //sel first axis
  GPIO_ResetBits(GPIOB,  GPIO_Pin_13);     //enable reading
  GPIO_ResetBits(GPIOB,  GPIO_Pin_12);
  GPIO_SetBits(GPIOB,  GPIO_Pin_9);        //sel MSB
  abData = (GPIOC->IDR)&0x00FF;
  GPIO_SetBits(GPIOB,  GPIO_Pin_12);
  GPIO_SetBits(GPIOB,  GPIO_Pin_9);        //sel 2ND
  abData = (abData<<8) + ((GPIOC->IDR)&0x00FF);
  GPIO_ResetBits(GPIOB,  GPIO_Pin_12);
  GPIO_ResetBits(GPIOB,  GPIO_Pin_9);      //sel 3RD
  abData = (abData<<8) + ((GPIOC->IDR)&0x00FF);
  GPIO_SetBits(GPIOB,  GPIO_Pin_12);
  GPIO_ResetBits(GPIOB,  GPIO_Pin_9);      //sel LSB
  abData = (abData<<8) + ((GPIOC->IDR)&0x00FF);
  GPIO_SetBits(GPIOB,  GPIO_Pin_13);       //disable reading
  abData = abData>>2;                      //设置4倍频 DIV4
  if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)==0) {  //读取方向
    return (0-abData);
  }
  else {
    return (abData);
  }
}

//ABOEN     -> PB13
//ABSEL2    -> PB9
//ABSEL1    -> PB12
//ABD0~ABD7 -> PC0~PC7
////////////////////////////////////////////////////////////////////////////////
//左轮：编码方向相反 注意正负号 与正常相反
////////////////////////////////////////////////////////////////////////////////
signed long halABYDecoderReadData(void) {
  signed long abData=0;
  GPIO_SetBits(GPIOB,  GPIO_Pin_8);        //sel 2nd axis
  GPIO_ResetBits(GPIOB,  GPIO_Pin_13);     //enable reading
  GPIO_ResetBits(GPIOB,  GPIO_Pin_12);
  GPIO_SetBits(GPIOB,  GPIO_Pin_9);        //sel MSB
  abData = (GPIOC->IDR)&0x00FF;
  GPIO_SetBits(GPIOB,  GPIO_Pin_12);
  GPIO_SetBits(GPIOB,  GPIO_Pin_9);        //sel 2ND
  abData = (abData<<8) + ((GPIOC->IDR)&0x00FF);
  GPIO_ResetBits(GPIOB,  GPIO_Pin_12);
  GPIO_ResetBits(GPIOB,  GPIO_Pin_9);      //sel 3RD
  abData = (abData<<8) + ((GPIOC->IDR)&0x00FF);
  GPIO_SetBits(GPIOB,  GPIO_Pin_12);
  GPIO_ResetBits(GPIOB,  GPIO_Pin_9);      //sel LSB
  abData = (abData<<8) + ((GPIOC->IDR)&0x00FF);
  GPIO_SetBits(GPIOB,  GPIO_Pin_13);       //disable reading
  abData = abData>>2;                      //设置4倍频 DIV4
  if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)==0) {  //读取方向： 0=>正方向， 1=>负方向
    return (abData);
  }
  else {
    return (0-abData);
  }
}

void halABClearABXInhibitLogic(void) {
  //ABRSTNX -> PB15
  GPIO_ResetBits(GPIOB,  GPIO_Pin_15);
  GPIO_SetBits(GPIOB,  GPIO_Pin_15);
}

//ABRSTNX -> PB14
void halABClearABYInhibitLogic(void) {
  GPIO_ResetBits(GPIOB,  GPIO_Pin_14);
  GPIO_SetBits(GPIOB,  GPIO_Pin_14);
}

 