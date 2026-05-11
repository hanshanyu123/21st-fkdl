#ifndef CODE_USE_IMG_H_
#define CODE_USE_IMG_H_

#include "zf_common_headfile.h"

void use_img_init(void);
void use_img(void);
uint8 use_img_is_ready(void);
uint8 use_img_snapshot_centerpoint_target(void);
uint8 use_img_snapshot_mid_line(uint8 y);
uint32 use_img_snapshot_l_data_statics(void);
uint32 use_img_snapshot_r_data_statics(void);
uint8 use_img_snapshot_phase(void);

#endif
