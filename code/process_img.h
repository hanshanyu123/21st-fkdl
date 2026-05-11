#ifndef CODE_PROCESS_IMG_H_
#define CODE_PROCESS_IMG_H_

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x_double.h"

#define XM MT9V03X_1_W
#define YM MT9V03X_1_H
#define XX (XM - 1)
#define YY (YM - 1)

#define Deal_Left   2
#define Deal_Right  (XX - 2)
#define Deal_Bottom 5
#define Deal_Top    (YY - 8)

#define Lost_Left   (Deal_Left - 2)
#define Lost_Right  (Deal_Right + 2)
#define Lost_Bottom (Deal_Bottom - 2)
#define Lost_Top    (Deal_Top + 2)

#define IMG_W XM
#define IMG_H YM
#define _IMG_W (IMG_W - 1)
#define _IMG_H (IMG_H - 1)
#define IMG_SIZE (IMG_W * IMG_H)
#define MAP_SIZE (YM * XM)
#define USE_num (YM * 4)

#define Black        0
#define White        255
#define Control_line 1
#define Left_line    2
#define Right_line   3
#define Lost_line    4
#define Judge_line   5
#define Make_up      6
#define LandR        7

enum ImgFlag
{
    straightlineS,
    straightlineL,
    curve,
    annulus_l,
    annulus_r,
    crossroads,
    startline,
    disappear,
};

typedef enum Elements_Mode
{
    NO,
    YES,
} Elements_Mode;

typedef enum Elements_Lock
{
    Empty_,
    Annulus_,
    Disappear_,
    Stopline_,
} Elements_Lock;

typedef enum Annulus_Mode
{
    Differential,
    Tracing,
} Annulus_Mode;

typedef enum RUN_Dir
{
    Left,
    Right,
} RUN_Dir;

extern enum ImgFlag IF;
extern enum ImgFlag IF_L;
extern volatile Elements_Mode elements_Mode;
extern volatile Elements_Lock elements_Lock[10];
extern volatile Annulus_Mode Annulus_mode;
extern volatile RUN_Dir run_dir;
extern int elements_index;

extern uint8_t nowThreshold;
extern uint8 close_Threshold;
extern uint8 far_Threshold;
extern uint8 mid_Threshold;
extern int minGrayscale;
extern int maxGrayscale;
extern uint8_t minThreshold;
extern uint8_t maxThreshold;
extern int minGray;
extern int maxGray;
extern uint8 img_threshold_group[3];
extern uint16 EXP_TIME;
extern int UD_OFFSET_DEF;

extern uint8 imgOSTU[YM][XM];
#define imgGray  mt9v03x_image_1
#define imgTran  mt9v03x_image_2
#define img2wifi imgOSTU
extern uint8 white_num_row[YM];
extern uint8 white_num_col[XM];
extern uint8 mid_line[YM];
extern uint8 mid_line_valid;

extern uint8 start_center;
extern uint8 start_center_x;
extern uint8 start_center_y;
extern uint8 end_center_x;
extern uint8 end_center_y;
extern uint8 start_point_l;
extern uint8 start_point_r;

extern int points_l[USE_num][2];
extern int points_r[USE_num][2];
extern float dir_r[USE_num];
extern float dir_l[USE_num];
extern uint32 r_data_statics;
extern uint32 l_data_statics;
extern uint8 hightest;
extern uint16 leftmost[2];
extern uint16 rightmost[2];
extern uint16 topmost[2];

extern uint8 l_lost_tip;
extern uint8 r_lost_tip;
extern uint8 t_lost_tip;
extern uint8 b_lost_tip;
extern uint8 b_lost_num;
extern uint8 t_lost_num;
extern uint8 l_lost_num;
extern uint8 r_lost_num;
extern uint8 Stop_line;
extern uint8 dis_Solidline;

extern uint16 b_center[3];
extern uint16 l_center[3];
extern uint16 r_center[3];
extern uint16 t_center[3];
extern float white_width[YM];
extern float Length_5cm[YM];
extern float k1[YM];
extern float k2[YM];
extern uint8 Make_up_num[YM];
extern uint8 white_row_max[2];
extern uint8 white_row_min[2];
extern uint8 white_col_max[2];
extern uint8 white_col_min[2];

extern uint8 stopline_flag;
extern uint8 stop_num;
extern uint8 Annulus_flag;
extern float Annulus_Zero;
extern float Annulus_Yaw;
extern int Annulus_Curvature[5];
extern int Annulus_num;
extern uint8 dealimg_finish_flag;
extern int deal_runing;
extern int dealimg_time;
extern uint8 disappear_flag;
extern int disappear_num;
extern int disappear_total;
extern int Disappear_angle_L[3];
extern int Disappear_angle_R[3];
extern float Start_angle;
extern float Length;
extern float Disappear_Length;
extern float Disappear_Zero;
extern float Disappear_Yaw;
extern int Disappear_Dir_L[3][3];
extern int Disappear_Dir_R[3][3];
extern float Out_Annulus_Length;
extern float Stopline_Position;
extern float Stopline_Judge;
extern float Stop_Length;
extern int Curvature;

extern int EXTERN_sum[16];
extern int EXTERN_ROW[16][YM];
extern int EXTERN_L;
extern int EXTERN_R;
extern int Goto_U;
extern int Goto_D;
extern int Goto_L;
extern int Goto_R;
extern int UD;
extern int UL;
extern int UR;
extern int DL;
extern int DR;
extern int LR;
extern int UDL;
extern int UDR;
extern int ULR;
extern int DLR;
extern int UDLR;

void standard(void);
uint8 getOSTUThreshold(void);
void Get_imgOSTU(void);
uint8 Dashedline_Makeup(void);
uint8 get_start_point(void);
uint8 search_l_r(uint8 start_l_x, uint8 start_l_y, uint8 start_r_x, uint8 start_r_y);
void Get_lost_tip(uint8 length);
void Get_start_center(void);
uint8 Get_top_straightline(void);
void Get_curve_line(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 n);
uint8 Left_curve_line(void);
uint8 Right_curve_line(void);
uint8 Get_Annulus(void);
uint8 Go_Annulus(void);
uint8 Deal_crossroads(void);
uint8 Get_stopline(void);
uint8 Disappear_detection(void);
uint8 Deal_disappear(void);
void ProcessImg_UpdateMotion10ms(int16 speed_l, int16 speed_r);

#endif
