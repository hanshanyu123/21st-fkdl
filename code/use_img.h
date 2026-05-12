#ifndef _USE_IMG_H_
#define _USE_IMG_H_

#include "zf_common_headfile.h"

typedef enum
{
    straight = 0,
    curve = 1
} use_img_phase_enum;

void use_img_init(void);
void use_img(void);

uint8 use_img_is_ready(void);
uint8 use_img_snapshot_phase(void);
uint8 use_img_snapshot_l_data_statics(void);
uint8 use_img_snapshot_r_data_statics(void);
uint8 use_img_snapshot_centerpoint_target(void);

#endif
