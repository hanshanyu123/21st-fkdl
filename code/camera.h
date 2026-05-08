#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_device_ips200.h"

extern int16 line_mid[MT9V03X_1_H];
extern int16 track_offset;
extern uint8 junction_type_from_camera;

void cam_init(void);
void image_process_task(void);
void image_display_task(void);
int16 Camera_GetTrackOffset(void);
uint8 Camera_GetLineLostCount(void);
uint8 Camera_GetThreshold(void);
uint8 Camera_GetThresholdNear(void);
uint8 Camera_GetThresholdMid(void);
uint8 Camera_GetThresholdFar(void);
uint8 Camera_GetWhiteRow(uint8 row);
uint8 Camera_GetWhiteCol(uint8 col);
int16 Camera_GetCenterLine(uint8 row);

#endif
