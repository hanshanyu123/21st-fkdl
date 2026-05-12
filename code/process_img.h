#ifndef _PROCESS_IMG_H_
#define _PROCESS_IMG_H_

#include "zf_common_headfile.h"

#define XM 80
#define YM 60

extern uint8 imgGray[YM][XM];
extern uint8 img2wifi[YM][XM];
extern uint8 process_img_last_threshold;
extern float Length;

void ProcessImg_Init(void);
uint8 ProcessImg_ProcessFrame(void);
void ProcessImg_UpdateMotion10ms(int16 speed_l, int16 speed_r);

#endif
