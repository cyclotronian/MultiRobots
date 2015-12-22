#ifndef __HALABDECODER_H
#define __HALABDECODER_H

#include "stm32f10x.h"


//----������������ʽ-------------
#define  COUNT_MODE_X1    0
#define  COUNT_MODE_X2    1
#define  COUNT_MODE_X4    2

extern void halABDecoderInit(void);
extern signed long halABXDecoderReadData(void);
extern signed long halABYDecoderReadData(void);
extern void halABClearABXInhibitLogic(void);
extern void halABClearABYInhibitLogic(void);


#endif

