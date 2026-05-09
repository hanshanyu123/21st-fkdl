#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_device_ips200.h"

extern int16 line_mid[MT9V03X_1_H];
extern int16 track_offset;
extern uint8 junction_type_from_camera;

#define VISION_NORMAL       0
#define VISION_COMPONENT    1
#define VISION_CORNER_LEFT  2
#define VISION_CORNER_RIGHT 3
#define VISION_LOST         4

#define VISION_PHASE_NORMAL          0
#define VISION_PHASE_COMPONENT_ENTER 1
#define VISION_PHASE_COMPONENT_LOCK  2
#define VISION_PHASE_COMPONENT_EXIT  3
#define VISION_PHASE_CORNER_L_ENTER  4
#define VISION_PHASE_CORNER_L_TURN   5
#define VISION_PHASE_CORNER_L_EXIT   6
#define VISION_PHASE_CORNER_R_ENTER  7
#define VISION_PHASE_CORNER_R_TURN   8
#define VISION_PHASE_CORNER_R_EXIT   9
#define VISION_PHASE_LOST            10

void cam_init(void);
void image_process_task(void);
void image_display_task(void);
int16 Camera_GetTrackOffset(void);
uint8 Camera_GetLineLostCount(void);
uint8 Camera_GetCornerType(void);
uint8 Camera_GetVisionState(void);
uint8 Camera_GetVisionPhase(void);
uint8 Camera_GetCornerDirection(void);
uint8 Camera_GetLeftLostCount(void);
uint8 Camera_GetRightLostCount(void);
uint8 Camera_GetTopLostCount(void);
uint8 Camera_GetBottomLostCount(void);
uint8 Camera_GetLeftLostBlockCount(void);
uint8 Camera_GetRightLostBlockCount(void);
uint8 Camera_GetTopLostBlockCount(void);
uint8 Camera_GetBottomLostBlockCount(void);
uint8 Camera_IsFrameReady(void);
uint8 Camera_GetLineConfidence(void);
uint8 Camera_GetComponentRowCount(void);
uint8 Camera_GetThreshold(void);
uint8 Camera_GetThresholdNear(void);
uint8 Camera_GetThresholdMid(void);
uint8 Camera_GetThresholdFar(void);
uint8 Camera_GetWhiteRow(uint8 row);
uint8 Camera_GetWhiteCol(uint8 col);
int16 Camera_GetCenterLine(uint8 row);

#endif
