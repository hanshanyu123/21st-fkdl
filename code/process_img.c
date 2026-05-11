#include "process_img.h"
#include <string.h>

#define PROCESS_THRESHOLD_MIN 40
#define PROCESS_THRESHOLD_MAX 160
#define PROCESS_CLOSE_OFFSET  0
#define PROCESS_MID_OFFSET    0
#define PROCESS_FAR_OFFSET    0
#define PROCESS_MM_PER_COUNT  0.003897f

enum ImgFlag IF = straightlineS;
enum ImgFlag IF_L = straightlineS;
volatile Elements_Mode elements_Mode = NO;
volatile Elements_Lock elements_Lock[10] = {Empty_};
volatile Annulus_Mode Annulus_mode = Differential;
volatile RUN_Dir run_dir = Right;
int elements_index = 0;

uint8_t nowThreshold = 0;
uint8 close_Threshold = 0;
uint8 far_Threshold = 0;
uint8 mid_Threshold = 0;
int minGrayscale = 40;
int maxGrayscale = 160;
uint8_t minThreshold = 70;
uint8_t maxThreshold = 160;
int minGray = 0;
int maxGray = 0;
uint8 img_threshold_group[3] = {0};
uint16 EXP_TIME = 255;
int UD_OFFSET_DEF = 0;
static uint32 process_histogram[256] = {0};
static uint8 previousThreshold = 100;

uint8 imgOSTU[YM][XM];
uint8 white_num_row[YM];
uint8 white_num_col[XM];
uint8 mid_line[YM];
uint8 mid_line_valid = 0;
uint8 Make_up_num[YM];
uint8 white_row_max[2] = {0};
uint8 white_row_min[2] = {0};
uint8 white_col_max[2] = {0};
uint8 white_col_min[2] = {0};

uint8 start_center = XM / 2;
uint8 start_center_x = XM / 2;
uint8 start_center_y = Deal_Bottom;
uint8 end_center_x = XM / 2;
uint8 end_center_y = Deal_Top;
uint8 start_point_l = Deal_Left;
uint8 start_point_r = Deal_Right;

int points_l[USE_num][2];
int points_r[USE_num][2];
float dir_r[USE_num];
float dir_l[USE_num];
uint32 r_data_statics = 0;
uint32 l_data_statics = 0;
uint8 hightest = 0;
uint16 leftmost[2] = {0};
uint16 rightmost[2] = {0};
uint16 topmost[2] = {0};

uint8 l_lost_tip = 0;
uint8 r_lost_tip = 0;
uint8 t_lost_tip = 0;
uint8 b_lost_tip = 0;
uint8 b_lost_num = 0;
uint8 t_lost_num = 0;
uint8 l_lost_num = 0;
uint8 r_lost_num = 0;
uint8 Stop_line = 0;
uint8 dis_Solidline = 0;

uint16 b_center[3] = {0};
uint16 l_center[3] = {0};
uint16 r_center[3] = {0};
uint16 t_center[3] = {0};

float white_width[YM];
float Length_5cm[YM];
float k1[YM];
float k2[YM];

uint8 stopline_flag = 0;
uint8 stop_num = 0;
uint8 Annulus_flag = 0;
float Annulus_Zero = 0.0f;
float Annulus_Yaw = 0.0f;
int Annulus_Curvature[5] = {10, 10, 10, 10, 10};
int Annulus_num = 0;
uint8 dealimg_finish_flag = 0;
int deal_runing = 0;
int dealimg_time = 0;
uint8 disappear_flag = 0;
int disappear_num = 0;
int disappear_total = 3;
int Disappear_angle_L[3] = {0};
int Disappear_angle_R[3] = {0};
float Start_angle = 0.0f;
float Length = 0.0f;
float Disappear_Length = 0.0f;
float Disappear_Zero = 0.0f;
float Disappear_Yaw = 0.0f;
int Disappear_Dir_L[3][3] = {{0}};
int Disappear_Dir_R[3][3] = {{0}};
float Out_Annulus_Length = 0.0f;
float Stopline_Position = 0.0f;
float Stopline_Judge = 300.0f;
float Stop_Length = 0.0f;
int Curvature = 10;

int EXTERN_sum[16] = {0};
int EXTERN_ROW[16][YM] = {{0}};
int EXTERN_L = 0;
int EXTERN_R = 0;
int Goto_U = 1;
int Goto_D = 2;
int Goto_L = 4;
int Goto_R = 8;
int UD = 3;
int UL = 5;
int UR = 9;
int DL = 6;
int DR = 10;
int LR = 12;
int UDL = 7;
int UDR = 11;
int ULR = 13;
int DLR = 14;
int UDLR = 15;

static int8_t seeds_l[8][2] = {
    {0, -1}, {-1, -1}, {-1, 0}, {-1, 1},
    {0, 1}, {1, 1}, {1, 0}, {1, -1}
};
static int8_t seeds_r[8][2] = {
    {0, -1}, {1, -1}, {1, 0}, {1, 1},
    {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}
};

static uint8 white_width_initialized = 0;

static int abs_i32(int value)
{
    return (value >= 0) ? value : -value;
}

static uint8 clamp_threshold(int value)
{
    if(value < 1) return 1;
    if(value > 254) return 254;
    return (uint8)value;
}

static uint8 is_dark_value(uint8 value)
{
    return (value == Black || value == Lost_line);
}

static uint8 pixel_at(int x, int y)
{
    if(x < 0 || x >= XM || y < 0 || y >= YM)
    {
        return Black;
    }
    return imgOSTU[y][x];
}

static void init_white_width_table(void)
{
    static const float grass_white_width[70] = {
        17,17,17,17,17,16,16,16,16,16,
        16,16,16,15,15,15,15,14,14,14,
        14,14,14,13,13,13,13,13,13,13,
        12,12,12,12,12,11,11,11,11,10,
        10,10,10,10,10,10,10,10,10,9,
        9,9,9,9,9,8,8,8,8,8,
        7,7,7,7,6,6,6,6,5,5
    };
    static const float grass_length_5cm[70] = {
        30,29,28,28,27,27,26,26,25,25,
        25,25,25,25,24,24,24,23,23,23,
        22,22,22,22,21,21,21,20,20,20,
        19,19,19,18,18,18,17,17,16,16,
        15,15,15,14,14,14,13,13,12,12,
        11,11,11,11,11,10,10,10,9,9,
        9,8,8,8,8,7,7,7,7,7
    };
    static const float grass_k2[70] = {
        0.00f,0.28f,0.57f,0.85f,1.14f,1.45f,1.71f,1.99f,2.28f,2.56f,
        2.85f,3.13f,3.42f,3.70f,3.99f,4.27f,4.56f,4.84f,5.13f,5.41f,
        5.70f,5.98f,6.27f,6.55f,6.84f,7.12f,7.41f,7.69f,7.98f,8.26f,
        8.55f,8.83f,9.12f,9.40f,9.69f,9.97f,10.26f,10.54f,10.83f,11.11f,
        11.28f,11.57f,11.85f,12.14f,12.43f,12.71f,13.00f,13.28f,13.57f,13.86f,
        14.14f,14.43f,14.71f,15.00f,15.29f,15.57f,15.86f,16.14f,16.43f,16.72f,
        17.00f,17.29f,17.57f,17.86f,18.15f,18.43f,18.72f,19.01f,19.29f,20.00f
    };
    const int grass_rows = 70;

    if(white_width_initialized)
    {
        return;
    }

    for(int y = 0; y < YM; y++)
    {
        float mapped = ((float)y * (float)(grass_rows - 1)) / (float)(YM - 1);
        int idx0 = (int)mapped;
        int idx1 = idx0 + 1;
        float frac = mapped - (float)idx0;
        float width;
        float len5;
        float k2_value;

        if(idx1 >= grass_rows)
        {
            idx1 = grass_rows - 1;
        }

        width = grass_white_width[idx0] + (grass_white_width[idx1] - grass_white_width[idx0]) * frac;
        len5 = grass_length_5cm[idx0] + (grass_length_5cm[idx1] - grass_length_5cm[idx0]) * frac;
        k2_value = grass_k2[idx0] + (grass_k2[idx1] - grass_k2[idx0]) * frac;

        white_width[y] = width;
        Length_5cm[y] = len5;
        k1[y] = 25.0f / width;
        k2[y] = k2_value;
    }

    white_width_initialized = 1;
}

static void reset_process_state(void)
{
    memset(imgOSTU, Black, sizeof(imgOSTU));
    memset(white_num_row, 0, sizeof(white_num_row));
    memset(white_num_col, 0, sizeof(white_num_col));
    memset(points_l, 0, sizeof(points_l));
    memset(points_r, 0, sizeof(points_r));
    memset(dir_l, 0, sizeof(dir_l));
    memset(dir_r, 0, sizeof(dir_r));
    memset(mid_line, XM / 2, sizeof(mid_line));
    mid_line_valid = 0;
    memset(Make_up_num, 0, sizeof(Make_up_num));
    memset(b_center, 0, sizeof(b_center));
    memset(l_center, 0, sizeof(l_center));
    memset(r_center, 0, sizeof(r_center));
    memset(t_center, 0, sizeof(t_center));
    memset(EXTERN_sum, 0, sizeof(EXTERN_sum));
    memset(EXTERN_ROW, 0, sizeof(EXTERN_ROW));
    memset(white_row_max, 0, sizeof(white_row_max));
    memset(white_row_min, 0, sizeof(white_row_min));
    memset(white_col_max, 0, sizeof(white_col_max));
    memset(white_col_min, 0, sizeof(white_col_min));

    l_data_statics = 0;
    r_data_statics = 0;
    hightest = 0;
    leftmost[0] = 0;
    leftmost[1] = 0;
    rightmost[0] = 0;
    rightmost[1] = 0;
    topmost[0] = 0;
    topmost[1] = 0;

    l_lost_tip = 0;
    r_lost_tip = 0;
    t_lost_tip = 0;
    b_lost_tip = 0;
    b_lost_num = 0;
    t_lost_num = 0;
    l_lost_num = 0;
    r_lost_num = 0;
    Stop_line = 0;
    dis_Solidline = 0;
    stopline_flag = 0;
    stop_num = 0;
    Annulus_flag = 0;
    Annulus_Zero = 0.0f;
    Annulus_Yaw = 0.0f;
    Annulus_num = 0;
    disappear_flag = 0;
    disappear_num = 0;
    disappear_total = 3;
    elements_index = 0;
    Out_Annulus_Length = 0.0f;
    Stopline_Position = 0.0f;
    Stop_Length = 0.0f;
    dealimg_finish_flag = 0;
    deal_runing = 0;
    dealimg_time = 0;
    EXTERN_L = 0;
    EXTERN_R = 0;

    start_center = XM / 2;
    start_center_x = XM / 2;
    start_center_y = Deal_Bottom;
    end_center_x = XM / 2;
    end_center_y = Deal_Top;
    start_point_l = Deal_Left;
    start_point_r = Deal_Right;
}

static void apply_process_frame(void)
{
    for(int x = 0; x <= XX; x++)
    {
        if(imgOSTU[Deal_Bottom - 1][x] == White || imgOSTU[Deal_Bottom - 2][x] == White)
        {
            imgOSTU[Deal_Bottom][x] = White;
        }
        imgOSTU[Deal_Bottom - 1][x] = Black;
        imgOSTU[Deal_Bottom - 2][x] = Black;

        if(imgOSTU[Deal_Top + 1][x] == White || imgOSTU[Deal_Top + 2][x] == White)
        {
            imgOSTU[Deal_Top][x] = White;
        }
        imgOSTU[Deal_Top + 1][x] = Black;
        imgOSTU[Deal_Top + 2][x] = Black;
    }

    for(int y = 0; y <= Deal_Top; y++)
    {
        if(imgOSTU[y][Deal_Left - 1] == White || imgOSTU[y][Deal_Left - 2] == White)
        {
            imgOSTU[y][Deal_Left] = White;
        }
        imgOSTU[y][Deal_Left - 1] = Black;
        imgOSTU[y][Deal_Left - 2] = Black;

        if(imgOSTU[y][Deal_Right + 1] == White || imgOSTU[y][Deal_Right + 2] == White)
        {
            imgOSTU[y][Deal_Right] = White;
        }
        imgOSTU[y][Deal_Right + 1] = Black;
        imgOSTU[y][Deal_Right + 2] = Black;
    }
}

static uint8 draw_control_line(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 color)
{
    uint8 black_num = 0;
    float k;

    if(y2 <= y1)
    {
        return 0;
    }

    k = ((float)x2 - (float)x1) / ((float)y2 - (float)y1);
    for(uint8 y = y1; y <= y2; y++)
    {
        int x = (int)((float)x1 + k * (float)(y - y1));

        if(x < 0) x = 0;
        if(x > XX) x = XX;
        mid_line[y] = (uint8)x;
        if(imgOSTU[y][x] != White)
        {
            black_num++;
        }
        imgOSTU[y][x] = color;
    }

    mid_line_valid = 1;
    return black_num;
}

void standard(void)
{
    init_white_width_table();
}

static void getGrayscaleHistogram(void)
{
    memset(process_histogram, 0, sizeof(process_histogram));
    minGray = 255;
    maxGray = 0;

    for(int y = IMG_H - YM; y < IMG_H; y++)
    {
        for(int x = 0; x < IMG_W; x++)
        {
            uint8 gray = imgGray[y][x];

            if(gray < minGray)
            {
                minGray = gray;
            }
            if(gray > maxGray)
            {
                maxGray = gray;
            }
            process_histogram[gray]++;
        }
    }
}

uint8 getOSTUThreshold(void)
{
    uint32 sum = 0;
    uint32 valueSum = 0;
    uint32 lowSum[256] = {0};
    uint32 lowValueSum[256] = {0};
    uint64 sigma = 0;
    uint64 maxSigma = 0;
    uint8 min = 0;
    uint8 max = 255;

    getGrayscaleHistogram();
    min = (uint8)minGray;
    max = (uint8)maxGray;

    if(max < minThreshold)
    {
        nowThreshold = minThreshold;
        previousThreshold = nowThreshold;
        return nowThreshold;
    }
    if(min > maxThreshold)
    {
        nowThreshold = maxThreshold;
        previousThreshold = nowThreshold;
        return nowThreshold;
    }

    if(min < minGrayscale)
    {
        min = (uint8)minGrayscale;
    }
    if(max > maxGrayscale)
    {
        max = (uint8)maxGrayscale;
    }
    if(max <= min)
    {
        nowThreshold = previousThreshold;
        return nowThreshold;
    }

    for(uint16 i = min; i <= max; i++)
    {
        sum += process_histogram[i];
        valueSum += process_histogram[i] * i;
        lowSum[i] = sum;
        lowValueSum[i] = valueSum;
    }

    if(sum == 0)
    {
        nowThreshold = previousThreshold;
        return nowThreshold;
    }

    nowThreshold = previousThreshold;
    for(uint16 i = min; i <= max; i++)
    {
        uint32 countLow = lowSum[i];
        uint32 countHigh = sum - countLow;
        uint32 valueLow = lowValueSum[i];
        uint32 valueHigh = valueSum - valueLow;
        float w1;
        float w2;
        int32 u1;
        int32 u2;
        int32 delta;

        if(countLow == 0 || countHigh == 0)
        {
            continue;
        }

        w1 = (float)countLow / (float)sum;
        w2 = 1.0f - w1;
        u1 = (int32)(valueLow / countLow);
        u2 = (int32)(valueHigh / countHigh);
        delta = u1 - u2;
        sigma = (uint64)(w1 * w2 * (float)(delta * delta));

        if(sigma >= maxSigma)
        {
            maxSigma = sigma;
            nowThreshold = (uint8)i;
        }
        else
        {
            break;
        }
    }

    if(nowThreshold < minThreshold)
    {
        nowThreshold = minThreshold;
    }
    if(nowThreshold > maxThreshold)
    {
        nowThreshold = maxThreshold;
    }
    previousThreshold = nowThreshold;
    return nowThreshold;
}

void Get_imgOSTU(void)
{
    static const int loop_dir[8][2] = {
        {0, 1}, {1, 1}, {1, 0}, {1, -1},
        {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
    };
    int zone = 0;

    reset_process_state();
    nowThreshold = getOSTUThreshold();
    close_Threshold = PROCESS_CLOSE_OFFSET;
    mid_Threshold = PROCESS_MID_OFFSET;
    far_Threshold = PROCESS_FAR_OFFSET;
    img_threshold_group[0] = clamp_threshold((int)nowThreshold + close_Threshold);
    img_threshold_group[1] = clamp_threshold((int)nowThreshold - mid_Threshold);
    img_threshold_group[2] = clamp_threshold((int)nowThreshold - far_Threshold);

    for(int i = 0; i <= YY; i++)
    {
        if(i == YY / 3)
        {
            zone = 1;
        }
        else if(i == YY * 2 / 3)
        {
            zone = 2;
        }

        for(int j = 0; j <= XX; j++)
        {
            if(imgGray[_IMG_H - i][j] >= img_threshold_group[zone])
            {
                int white_num = 0;
                int black_num = 0;

                for(int k = 0; k <= 8; k++)
                {
                    int x = j + (k < 8 ? loop_dir[k][0] : 0);
                    int y = i + (k < 8 ? loop_dir[k][1] : 0);

                    if(k == 8)
                    {
                        if(imgGray[_IMG_H - i][j] >= nowThreshold) white_num++;
                        else black_num++;
                        continue;
                    }

                    if(x < 0 || x > XX || y < 0 || y > YY)
                    {
                        white_num++;
                    }
                    else if(imgOSTU[y][x] == White)
                    {
                        white_num = 8;
                        break;
                    }
                    else if(imgGray[_IMG_H - y][x] >= nowThreshold)
                    {
                        white_num++;
                    }
                    else
                    {
                        black_num++;
                    }

                    if(black_num >= 2)
                    {
                        break;
                    }
                }

                if(white_num >= 8)
                {
                    imgOSTU[i][j] = White;
                    white_num_row[i]++;
                    white_num_col[j]++;

                    for(int k = 0; k <= 8; k++)
                    {
                        int x = j + (k < 8 ? loop_dir[k][0] : 0);
                        int y = i + (k < 8 ? loop_dir[k][1] : 0);

                        if(k == 8) continue;

                        if(x >= 0 && x <= XX && y >= 0 && y <= YY && imgOSTU[y][x] != White)
                        {
                            imgOSTU[y][x] = White;
                            white_num_row[y]++;
                            white_num_col[x]++;
                        }
                    }
                }
            }
        }
    }

}

uint8 Dashedline_Makeup(void)
{
    white_row_max[0] = 0;
    white_row_min[0] = XX;
    white_col_max[0] = 0;
    white_col_min[0] = XX;

    for(uint8 y = Deal_Bottom; y <= Deal_Top; y++)
    {
        if(white_num_row[y] > white_row_max[0])
        {
            white_row_max[0] = white_num_row[y];
            white_row_max[1] = y;
        }
        if(white_num_row[y] < white_row_min[0])
        {
            white_row_min[0] = white_num_row[y];
            white_row_min[1] = y;
        }
    }
    for(uint8 x = Deal_Left; x <= Deal_Right; x++)
    {
        if(white_num_col[x] > white_col_max[0])
        {
            white_col_max[0] = white_num_col[x];
            white_col_max[1] = x;
        }
        if(white_num_col[x] < white_col_min[0])
        {
            white_col_min[0] = white_num_col[x];
            white_col_min[1] = x;
        }
    }

    if(white_col_max[0] >= YY && white_row_max[0] < (uint8)white_width[0])
    {
        apply_process_frame();
        return 0;
    }

    memset(imgTran, Black, sizeof(mt9v03x_image_2));
    for(uint8 i = 0; i <= YY; i++)
    {
        for(uint8 j = 0; j <= XX; j++)
        {
            if(imgOSTU[i][j] == White)
            {
                int w_span = (int)(2.0f * white_width[i]);
                int h_span = (int)Length_5cm[i];
                int w_start = (j - w_span < 0) ? 0 : (j - w_span);
                int w_end = (j + w_span > XX) ? XX : (j + w_span);
                int h_start = (i - h_span < 0) ? 0 : (i - h_span);
                int h_end = (i + h_span > YY) ? YY : (i + h_span);

                if(j != XX)
                {
                    for(int w = j + 1; w <= w_end; w++)
                    {
                        if(imgOSTU[i][w] == Black)
                        {
                            imgTran[i][w] = (uint8)(imgTran[i][w] + Goto_R);
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                if(j != 0)
                {
                    for(int w = j - 1; w >= w_start; w--)
                    {
                        if(imgOSTU[i][w] == Black)
                        {
                            imgTran[i][w] = (uint8)(imgTran[i][w] + Goto_L);
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                if(i != YY)
                {
                    for(int h = i + 1; h <= h_end; h++)
                    {
                        if(imgOSTU[h][j] == Black)
                        {
                            imgTran[h][j] = (uint8)(imgTran[h][j] + Goto_D);
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                if(i != 0)
                {
                    for(int h = i - 1; h >= h_start; h--)
                    {
                        if(imgOSTU[h][j] == Black)
                        {
                            imgTran[h][j] = (uint8)(imgTran[h][j] + Goto_U);
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    memset(EXTERN_sum, 0, sizeof(EXTERN_sum));
    memset(EXTERN_ROW, 0, sizeof(EXTERN_ROW));
    memset(Make_up_num, 0, sizeof(Make_up_num));

    for(uint8 i = 0; i <= YY; i++)
    {
        for(uint8 j = 0; j <= XX; j++)
        {
            if(imgOSTU[i][j] == Black && imgTran[i][j] >= 1)
            {
                if(imgTran[i][j] == ULR || imgTran[i][j] == DLR || imgTran[i][j] == UDLR)
                {
                    imgOSTU[i][j] = LandR;
                }
                else
                {
                    imgOSTU[i][j] = Make_up;
                    Make_up_num[i]++;
                }

                EXTERN_sum[imgTran[i][j]]++;
                EXTERN_ROW[imgTran[i][j]][i]++;
            }
        }
    }

    EXTERN_L = 0;
    EXTERN_R = 0;
    {
        uint8 l_white = 0;
        uint8 r_white = 0;
        uint8 l_end = Deal_Left;
        uint8 r_end = Deal_Right;

        for(uint8 k = 0; k <= (uint8)white_width[Deal_Top]; k++)
        {
            l_white += white_num_col[k];
            if(l_white)
            {
                l_end = k;
                if(l_white >= 25) break;
            }
        }
        for(uint8 k = 0; k <= (uint8)white_width[Deal_Top]; k++)
        {
            r_white += white_num_col[XX - k];
            if(r_white)
            {
                r_end = (uint8)(XX - k);
                if(r_white >= 25) break;
            }
        }

        for(uint8 y = Deal_Bottom; y <= YY; y++)
        {
            for(uint8 k = Deal_Left; k <= l_end; k++)
            {
                if(imgOSTU[y][k] == White)
                {
                    imgOSTU[y][Deal_Left] = White;
                    EXTERN_L++;
                    white_num_row[y]++;
                    white_num_col[Deal_Left]++;
                    break;
                }
                else if(imgOSTU[y][k] == Black)
                {
                    break;
                }
            }
        }
        for(uint8 y = Deal_Bottom; y <= YY; y++)
        {
            for(int k = Deal_Right; k >= r_end; k--)
            {
                if(imgOSTU[y][k] == White)
                {
                    imgOSTU[y][Deal_Right] = White;
                    EXTERN_R++;
                    white_num_row[y]++;
                    white_num_col[Deal_Right]++;
                    break;
                }
                else if(imgOSTU[y][k] == Black)
                {
                    break;
                }
            }
        }
    }

    if(!Annulus_flag)
    {
        uint8 white_top[XM];
        memset(white_top, 0, sizeof(white_top));

        for(uint8 x = Deal_Left; x <= Deal_Right; x++)
        {
            for(uint8 y = Deal_Top; y <= YY; y++)
            {
                if(imgOSTU[y][x] == White)
                {
                    white_top[x] = 1;
                    break;
                }
                else if(imgOSTU[y][x] == Black)
                {
                    break;
                }
            }
        }
        for(uint8 x = Deal_Left; x <= Deal_Right; x++)
        {
            if(imgOSTU[Deal_Top][x] == White && white_top[x] == 1)
            {
                white_top[x] = 0;
                for(int w = x + 1; w <= Deal_Right; w++)
                {
                    if(white_top[w] == 1) white_top[w] = 0;
                    else break;
                }
                for(int w = x - 1; w >= Deal_Left; w--)
                {
                    if(white_top[w] == 1) white_top[w] = 0;
                    else break;
                }
            }
        }
        for(uint8 x = Deal_Left; x <= Deal_Right; x++)
        {
            if(white_top[x] == 1)
            {
                imgOSTU[Deal_Top][x] = White;
                imgOSTU[Deal_Top - 1][x] = White;
                imgOSTU[Deal_Top - 2][x] = White;
                white_num_row[Deal_Top]++;
                white_num_row[Deal_Top - 1]++;
                white_num_row[Deal_Top - 2]++;
                white_num_col[x]++;
            }
        }
    }

    apply_process_frame();
    return 1;
}

uint8 get_start_point(void)
{
    uint8 l_found = 0;
    uint8 r_found = 0;
    uint32 l_point = 0;
    uint32 r_point = 0;

    b_lost_tip = 0;
    start_center_y = 0;

    for(uint8 y = Deal_Bottom; y <= Deal_Bottom + 30 && y + 1 < YM; y++)
    {
        if(white_num_row[y] >= 5 && white_num_row[y + 1] >= 5)
        {
            start_center_y = y;

            for(uint8 x = 0; x < XX; x++)
            {
                if((imgOSTU[start_center_y][x + 1] == White || imgOSTU[start_center_y][x + 1] == Make_up) &&
                   imgOSTU[start_center_y][x] == Black)
                {
                    l_point = x;
                    l_found = 1;
                    break;
                }
            }

            for(int x = XX; x > 1; x--)
            {
                if((imgOSTU[start_center_y][x - 1] == White || imgOSTU[start_center_y][x - 1] == Make_up) &&
                   imgOSTU[start_center_y][x] == Black)
                {
                    r_point = (uint32)x;
                    r_found = 1;
                    break;
                }
            }

            if(l_found && r_found)
            {
                break;
            }
        }
    }

    if(!(l_found && r_found))
    {
        return 0;
    }

    for(uint8 x = (uint8)l_point; x <= (uint8)r_point; x++)
    {
        if(imgOSTU[start_center_y][x] == White ||
           imgOSTU[start_center_y + 1][x] == White ||
           imgOSTU[Deal_Bottom][x] == White)
        {
            b_lost_tip++;
            imgOSTU[Lost_Bottom][x] = Lost_line;
        }
    }

    start_center_x = (uint8)((l_point + r_point) / 2);
    start_center = start_center_x;
    start_point_l = (uint8)l_point;
    start_point_r = (uint8)r_point;
    return 1;
}

uint8 search_l_r(uint8 start_l_x, uint8 start_l_y, uint8 start_r_x, uint8 start_r_y)
{
    int center_point_l[2];
    int center_point_r[2];
    int temp_l[8][2];
    int temp_r[8][2];

    hightest = 0;
    leftmost[0] = Deal_Right;
    rightmost[0] = Deal_Left;
    topmost[1] = 0;

    l_lost_tip = 0;
    r_lost_tip = 0;
    t_lost_tip = 0;
    l_data_statics = 0;
    r_data_statics = 0;

    center_point_l[0] = start_l_x;
    center_point_l[1] = start_l_y;
    center_point_r[0] = start_r_x;
    center_point_r[1] = start_r_y;

    points_l[0][0] = center_point_l[0];
    points_l[0][1] = center_point_l[1];
    points_r[0][0] = center_point_r[0];
    points_r[0][1] = center_point_r[1];

    {
        uint32 break_flag = USE_num;

        while(break_flag--)
    {
        l_data_statics++;
        memset(temp_l, 0, sizeof(temp_l));
        {
            uint8 index_l = 0;

            for(uint8 i = 0; i < 8; i++)
            {
                int x0 = center_point_l[0] + seeds_l[i][0];
                int y0 = center_point_l[1] + seeds_l[i][1];
                int x1 = center_point_l[0] + seeds_l[(i + 1) & 7][0];
                int y1 = center_point_l[1] + seeds_l[(i + 1) & 7][1];

                if(is_dark_value(pixel_at(x0, y0)) &&
                   !is_dark_value(pixel_at(x1, y1)))
                {
                    temp_l[index_l][0] = x0;
                    temp_l[index_l][1] = y0;
                    dir_l[l_data_statics - 1] = (float)i;
                    index_l++;
                }

                if(index_l > 0)
                {
                    center_point_l[0] = temp_l[0][0];
                    center_point_l[1] = temp_l[0][1];
                    for(uint8 j = 0; j < index_l; j++)
                    {
                        if(center_point_l[1] < temp_l[j][1])
                        {
                            center_point_l[0] = temp_l[j][0];
                            center_point_l[1] = temp_l[j][1];
                            dir_l[l_data_statics - 1] = (float)i;
                        }
                    }
                }
            }
        }

        if(center_point_l[0] == Deal_Left - 1 &&
           pixel_at(Deal_Left, center_point_l[1]) == White &&
           pixel_at(Lost_Left, center_point_l[1]) != Lost_line)
        {
            l_lost_tip++;
            imgOSTU[center_point_l[1]][Lost_Left] = Lost_line;
        }
        else if(center_point_l[0] == Deal_Right + 1 &&
                pixel_at(Deal_Right, center_point_l[1]) == White &&
                pixel_at(Lost_Right, center_point_l[1]) != Lost_line)
        {
            r_lost_tip++;
            imgOSTU[center_point_l[1]][Lost_Right] = Lost_line;
        }
        else if(center_point_l[1] == Deal_Top + 1 &&
                pixel_at(center_point_l[0], Deal_Top) == White &&
                pixel_at(center_point_l[0], Lost_Top) != Lost_line)
        {
            t_lost_tip++;
            imgOSTU[Lost_Top][center_point_l[0]] = Lost_line;
        }

        if(center_point_l[1] > hightest)
        {
            hightest = (uint8)center_point_l[1];
            topmost[0] = (uint16)center_point_l[0];
            topmost[1] = (uint16)center_point_l[1];
        }
        if(center_point_l[0] < (int)leftmost[0])
        {
            leftmost[0] = (uint16)center_point_l[0];
            leftmost[1] = (uint16)center_point_l[1];
        }

        r_data_statics++;
        memset(temp_r, 0, sizeof(temp_r));
        {
            uint8 index_r = 0;

            for(uint8 i = 0; i < 8; i++)
            {
                int x0 = center_point_r[0] + seeds_r[i][0];
                int y0 = center_point_r[1] + seeds_r[i][1];
                int x1 = center_point_r[0] + seeds_r[(i + 1) & 7][0];
                int y1 = center_point_r[1] + seeds_r[(i + 1) & 7][1];

                if(is_dark_value(pixel_at(x0, y0)) &&
                   !is_dark_value(pixel_at(x1, y1)))
                {
                    temp_r[index_r][0] = x0;
                    temp_r[index_r][1] = y0;
                    dir_r[r_data_statics - 1] = (float)i;
                    index_r++;
                }

                if(index_r > 0)
                {
                    center_point_r[0] = temp_r[0][0];
                    center_point_r[1] = temp_r[0][1];
                    for(uint8 j = 0; j < index_r; j++)
                    {
                        if(center_point_r[1] < temp_r[j][1])
                        {
                            center_point_r[0] = temp_r[j][0];
                            center_point_r[1] = temp_r[j][1];
                            dir_r[r_data_statics - 1] = (float)i;
                        }
                    }
                }
            }
        }

        if(center_point_r[0] == Deal_Right + 1 &&
           pixel_at(Deal_Right, center_point_r[1]) == White &&
           pixel_at(Lost_Right, center_point_r[1]) != Lost_line)
        {
            r_lost_tip++;
            imgOSTU[center_point_r[1]][Lost_Right] = Lost_line;
        }
        else if(center_point_r[0] == Deal_Left - 1 &&
                pixel_at(Deal_Left, center_point_r[1]) == White &&
                pixel_at(Lost_Left, center_point_r[1]) != Lost_line)
        {
            l_lost_tip++;
            imgOSTU[center_point_r[1]][Lost_Left] = Lost_line;
        }
        else if(center_point_r[1] == Deal_Top + 1 &&
                pixel_at(center_point_r[0], Deal_Top) == White &&
                pixel_at(center_point_r[0], Lost_Top) != Lost_line)
        {
            t_lost_tip++;
            imgOSTU[Lost_Top][center_point_r[0]] = Lost_line;
        }

        if(center_point_r[1] > hightest)
        {
            hightest = (uint8)center_point_r[1];
            topmost[0] = (uint16)center_point_r[0];
            topmost[1] = (uint16)center_point_r[1];
        }
        if(center_point_r[0] > (int)rightmost[0])
        {
            rightmost[0] = (uint16)center_point_r[0];
            rightmost[1] = (uint16)center_point_r[1];
        }

        if(hightest == Deal_Top - 1)
        {
            for(uint8 x = Deal_Left; x <= Deal_Right; x++)
            {
                imgOSTU[Lost_Top][x] = Black;
                imgOSTU[Lost_Top - 1][x] = Black;
            }
        }

        if(l_data_statics < USE_num)
        {
            points_l[l_data_statics][0] = center_point_l[0];
            points_l[l_data_statics][1] = center_point_l[1];
        }
        if(r_data_statics < USE_num)
        {
            points_r[r_data_statics][0] = center_point_r[0];
            points_r[r_data_statics][1] = center_point_r[1];
        }

        if(r_data_statics >= 2 && l_data_statics >= 3)
        {
            if((points_r[r_data_statics][0] == points_r[r_data_statics - 1][0] &&
                points_r[r_data_statics][0] == points_r[r_data_statics - 2][0] &&
                points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] &&
                points_r[r_data_statics][1] == points_r[r_data_statics - 2][1]) ||
               (points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] &&
                points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] &&
                points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] &&
                points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1]))
            {
                break;
            }
        }

        if(abs_i32(points_r[r_data_statics][0] - points_l[l_data_statics][0]) <= 1 &&
           abs_i32(points_r[r_data_statics][1] - points_l[l_data_statics][1]) <= 1)
        {
            break;
        }

        if((l_data_statics > YY / 2 &&
            abs_i32((int)start_l_x - points_l[l_data_statics][0]) <= 1 &&
            abs_i32((int)start_l_y - points_l[l_data_statics][1]) <= 1) ||
           (r_data_statics > YY / 2 &&
            abs_i32((int)start_r_x - points_r[r_data_statics][0]) <= 1 &&
            abs_i32((int)start_r_y - points_r[r_data_statics][1]) <= 1))
        {
            break;
        }
    }
    }

    for(int i = (int)l_data_statics; i > 0; i--)
    {
        if(points_l[i][0] >= 0 && points_l[i][0] <= XX &&
           points_l[i][1] >= 0 && points_l[i][1] <= YY)
        {
            imgOSTU[points_l[i][1]][points_l[i][0]] = Left_line;
        }
    }
    for(int i = (int)r_data_statics; i > 0; i--)
    {
        if(points_r[i][0] >= 0 && points_r[i][0] <= XX &&
           points_r[i][1] >= 0 && points_r[i][1] <= YY)
        {
            imgOSTU[points_r[i][1]][points_r[i][0]] = Right_line;
        }
    }

    return (l_data_statics > 0 && r_data_statics > 0) ? 1 : 0;
}

void Get_lost_tip(uint8 length)
{
    dis_Solidline = 0;
    b_lost_num = 0;
    t_lost_num = 0;
    l_lost_num = 0;
    r_lost_num = 0;
    memset(b_center, 0, sizeof(b_center));
    memset(l_center, 0, sizeof(l_center));
    memset(r_center, 0, sizeof(r_center));
    memset(t_center, 0, sizeof(t_center));

    if(b_lost_tip)
    {
        for(uint8 x = Deal_Left; x <= Deal_Right; x++)
        {
            if(imgOSTU[Lost_Bottom][x] == Lost_line)
            {
                if(x <= Deal_Left + length || x >= Deal_Right - length ||
                   (x + length <= XX && imgOSTU[Lost_Bottom][x + length] == Lost_line))
                {
                    for(uint8 k = x; k <= XX; k++)
                    {
                        if(imgOSTU[Lost_Bottom][k] == Black)
                        {
                            b_center[b_lost_num] = (uint16)(((uint32)k - 1U - (uint32)x) / 2U + (uint32)x);
                            b_lost_num++;
                            x = k;
                            break;
                        }
                    }
                }
                else
                {
                    for(uint8 k = x; k <= x + length && k <= XX; k++)
                    {
                        if(imgOSTU[Lost_Bottom][k] == Lost_line)
                        {
                            imgOSTU[Lost_Bottom][k] = Black;
                            if(b_lost_tip > 0) b_lost_tip--;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
            if(b_lost_num == 2)
            {
                break;
            }
        }
    }

    if(l_lost_tip)
    {
        for(uint8 y = Deal_Bottom; y <= Deal_Top; y++)
        {
            if(imgOSTU[y][Lost_Left] == Lost_line)
            {
                if(y <= Deal_Bottom + length)
                {
                    for(uint8 x = Deal_Left; x <= Deal_Left + length; x++)
                    {
                        if(imgOSTU[Lost_Bottom][x] == Lost_line)
                        {
                            imgOSTU[Lost_Bottom][Lost_Left] = Lost_line;
                            for(; y <= Deal_Top / 2; y++)
                            {
                                if(imgOSTU[y][Lost_Left] == Lost_line)
                                {
                                    imgOSTU[y][Lost_Left] = Black;
                                    if(l_lost_tip > 0) l_lost_tip--;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                else
                {
                    uint8 start = y;
                    for(; y <= Lost_Top; y++)
                    {
                        if(imgOSTU[y][Lost_Left] == Black)
                        {
                            l_center[l_lost_num] = (uint16)(((uint32)y - 1U - (uint32)start) / 2U + (uint32)start);
                            l_lost_num++;
                            break;
                        }
                    }
                }
            }
            if(l_lost_num == 2)
            {
                break;
            }
        }
    }

    if(r_lost_tip)
    {
        for(uint8 y = Deal_Bottom; y <= Deal_Top; y++)
        {
            if(imgOSTU[y][Lost_Right] == Lost_line)
            {
                if(y <= Deal_Bottom + length)
                {
                    for(int x = Deal_Right; x >= Deal_Right - length; x--)
                    {
                        if(imgOSTU[Lost_Bottom][x] == Lost_line)
                        {
                            imgOSTU[Lost_Bottom][Lost_Right] = Lost_line;
                            for(; y <= Deal_Top / 2; y++)
                            {
                                if(imgOSTU[y][Lost_Right] == Lost_line)
                                {
                                    imgOSTU[y][Lost_Right] = Black;
                                    if(r_lost_tip > 0) r_lost_tip--;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                else
                {
                    uint8 start = y;
                    for(; y <= Lost_Top; y++)
                    {
                        if(imgOSTU[y][Lost_Right] == Black)
                        {
                            r_center[r_lost_num] = (uint16)(((uint32)y - 1U - (uint32)start) / 2U + (uint32)start);
                            r_lost_num++;
                            break;
                        }
                    }
                }
            }
            if(r_lost_num == 2)
            {
                break;
            }
        }
    }

    if(t_lost_tip)
    {
        uint16 center[5] = {0};
        uint16 num = 0;

        for(uint8 x = Deal_Left; x <= Deal_Right; x++)
        {
            if(imgOSTU[Lost_Top][x] == Lost_line)
            {
                if(x <= Deal_Left + length && l_lost_num)
                {
                    for(int y = Deal_Top; y >= Deal_Top - length; y--)
                    {
                        if(imgOSTU[y][Lost_Left] == Lost_line)
                        {
                            imgOSTU[Lost_Top][Lost_Left] = Lost_line;
                            for(int k = y; k >= YM / 2; k--)
                            {
                                if(imgOSTU[k][Lost_Left] == Lost_line)
                                {
                                    if(l_lost_tip > 0) l_lost_tip--;
                                    imgOSTU[k][Lost_Left] = Black;
                                }
                                else
                                {
                                    if(l_lost_num > 0) l_lost_num--;
                                    break;
                                }
                            }
                            for(uint8 k = x; k <= XX; k++)
                            {
                                if(imgOSTU[Lost_Top][k] == Black)
                                {
                                    center[num] = (uint16)(((uint32)k - 1U - (uint32)x) / 2U + (uint32)x);
                                    num++;
                                    x = k;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                else if(x + length >= XX || (x + length <= XX && imgOSTU[Lost_Top][x + length] == Lost_line))
                {
                    for(uint8 k = x; k <= XX; k++)
                    {
                        if(imgOSTU[Lost_Top][k] == Black)
                        {
                            center[num] = (uint16)(((uint32)k - 1U - (uint32)x) / 2U + (uint32)x);
                            num++;
                            x = k;

                            if(x >= Deal_Right - length && r_lost_num)
                            {
                                for(int y = Deal_Top; y >= Deal_Top - length; y--)
                                {
                                    if(imgOSTU[y][Lost_Right] == Lost_line)
                                    {
                                        imgOSTU[Lost_Top][Lost_Right] = Lost_line;
                                        for(int kk = y; kk >= YM / 2; kk--)
                                        {
                                            if(imgOSTU[kk][Lost_Right] == Lost_line)
                                            {
                                                if(r_lost_tip > 0) r_lost_tip--;
                                                imgOSTU[kk][Lost_Right] = Black;
                                            }
                                            else
                                            {
                                                if(r_lost_num > 0) r_lost_num--;
                                                break;
                                            }
                                        }
                                        break;
                                    }
                                }
                            }
                            break;
                        }
                    }
                }
                else
                {
                    for(uint8 k = x; k <= x + length && k <= XX; k++)
                    {
                        if(imgOSTU[Lost_Top][k] == Lost_line)
                        {
                            imgOSTU[Lost_Top][k] = Black;
                            if(t_lost_tip > 0) t_lost_tip--;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }

        if(num > 2)
        {
            uint8 get_center[2] = {0};

            for(uint8 k = 0; k <= XM / 2 - 1; k++)
            {
                for(uint8 i = 0; i <= 4; i++)
                {
                    if(center[i] == XM / 2 + k && t_lost_num < 2)
                    {
                        get_center[t_lost_num] = (uint8)(XM / 2 + k);
                        t_lost_num++;
                    }
                    if(center[i] == XM / 2 - k && t_lost_num < 2)
                    {
                        get_center[t_lost_num] = (uint8)(XM / 2 - k);
                        t_lost_num++;
                    }
                }
                if(t_lost_num >= 2)
                {
                    t_center[0] = (get_center[0] < get_center[1]) ? get_center[0] : get_center[1];
                    t_center[1] = (get_center[0] > get_center[1]) ? get_center[0] : get_center[1];
                    break;
                }
            }
        }
        else
        {
            t_lost_num = (uint8)num;
            t_center[0] = center[0];
            t_center[1] = center[1];
        }
    }
    else if(l_lost_tip + r_lost_tip == 0 && hightest >= Deal_Top)
    {
        int left_lost = 0;
        int left_end = Deal_Top;
        int right_lost = 0;
        int right_end = Deal_Top;

        dis_Solidline = 1;

        for(int y = Deal_Top; y >= Deal_Bottom; y--)
        {
            if(imgOSTU[y][Deal_Left] == Left_line || imgOSTU[y][Deal_Left] == Right_line)
            {
                for(int k = y; k >= Deal_Bottom; k--)
                {
                    if(imgOSTU[k][Deal_Left] != Black)
                    {
                        left_lost++;
                    }
                    else
                    {
                        if(k < YY / 2)
                        {
                            left_lost = 0;
                            break;
                        }
                        left_end = (y + k) / 2;
                        break;
                    }
                }
                break;
            }
        }

        for(int y = Deal_Top; y >= Deal_Bottom; y--)
        {
            if(imgOSTU[y][Deal_Right] == Left_line || imgOSTU[y][Deal_Right] == Right_line)
            {
                for(int k = y; k >= Deal_Bottom; k--)
                {
                    if(imgOSTU[k][Deal_Right] != Black)
                    {
                        right_lost++;
                    }
                    else
                    {
                        if(k < YY / 2)
                        {
                            right_lost = 0;
                            break;
                        }
                        right_end = (y + k) / 2;
                        break;
                    }
                }
                break;
            }
        }

        if(left_lost && !right_lost)
        {
            imgOSTU[left_end - 1][Lost_Left] = Lost_line;
            imgOSTU[left_end][Lost_Left] = Lost_line;
            imgOSTU[left_end + 1][Lost_Left] = Lost_line;
            l_center[0] = (uint16)left_end;
            l_lost_num = 1;
        }
        else if(right_lost && !left_lost)
        {
            imgOSTU[right_end - 1][Lost_Right] = Lost_line;
            imgOSTU[right_end][Lost_Right] = Lost_line;
            imgOSTU[right_end + 1][Lost_Right] = Lost_line;
            r_center[0] = (uint16)right_end;
            r_lost_num = 1;
        }
        else if(hightest >= Deal_Top)
        {
            int x_mid = topmost[0];
            int x_l = topmost[0];
            int x_r = topmost[0];
            int y_mid = topmost[1];

            for(int y = Deal_Top; y >= Deal_Bottom; y--)
            {
                if(imgOSTU[y][x_mid] == White)
                {
                    y_mid = y;
                    break;
                }
            }

            for(int x = x_mid; x >= Deal_Left; x--)
            {
                if(imgOSTU[y_mid][x] == White)
                {
                    imgOSTU[Lost_Top][x] = Lost_line;
                }
                else
                {
                    x_l = x;
                    break;
                }
            }
            for(int x = x_mid; x <= Deal_Right; x++)
            {
                if(imgOSTU[y_mid][x] == White)
                {
                    imgOSTU[Lost_Top][x] = Lost_line;
                }
                else
                {
                    x_r = x;
                    break;
                }
            }

            t_lost_num = 1;
            t_center[0] = (uint16)((x_l + x_r) / 2);
        }
    }
}

void Get_start_center(void)
{
    if(b_lost_tip >= (uint8)(white_width[Deal_Bottom] + 5.0f) && b_lost_num == 1)
    {
        uint32 max = 0;
        uint32 min = Deal_Top;
        uint32 mid = Deal_Top / 2;
        uint8 white_height[XM] = {0};

        for(uint8 x = start_point_l; x <= start_point_r; x++)
        {
            if(imgOSTU[Lost_Bottom][x] == Lost_line)
            {
                for(uint8 y = Deal_Bottom; y <= Deal_Top + 1; y++)
                {
                    if(imgOSTU[y][x] == Black || imgOSTU[y][x] == Left_line || imgOSTU[y][x] == Right_line)
                    {
                        white_height[x] = y;
                        if(max < y) max = y;
                        if(min > y) min = y;
                        break;
                    }
                }
            }
        }

        if(b_lost_tip >= (uint8)(2.0f * white_width[Deal_Bottom]))
        {
            mid = (max - min) * 2U / 3U + min;
        }
        else
        {
            mid = (max + min) / 2U;
        }

        for(uint8 x = start_point_l; x <= start_point_r; x++)
        {
            if(imgOSTU[Lost_Bottom][x] == Lost_line && white_height[x] < mid)
            {
                if(b_lost_tip > 0) b_lost_tip--;
                imgOSTU[Lost_Bottom][x] = Black;
            }
        }
    }
    else if(b_lost_num == 2)
    {
        uint8 l_point = 0;
        uint8 r_point = XM;
        float l_num = 0.0f;
        float r_num = 0.0f;
        float l_sum = 0.0f;
        float r_sum = 0.0f;

        for(uint8 x = start_point_l; x <= start_point_r; x++)
        {
            if(imgOSTU[Lost_Bottom][x] == Lost_line)
            {
                l_point = x;
                for(uint8 j = l_point; j <= Deal_Right; j++)
                {
                    if(imgOSTU[Lost_Bottom][j] == Lost_line)
                    {
                        for(uint8 k = Deal_Bottom + 1; k <= Deal_Top + 1; k++)
                        {
                            if(imgOSTU[k][j] == Left_line || imgOSTU[k][j] == Right_line)
                            {
                                l_num += 1.0f;
                                l_sum += (float)k;
                                break;
                            }
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                break;
            }
        }

        for(int x = start_point_r; x >= start_point_l; x--)
        {
            if(imgOSTU[Lost_Bottom][x] == Lost_line)
            {
                r_point = (uint8)x;
                for(int j = x; j >= Deal_Left; j--)
                {
                    if(imgOSTU[Lost_Bottom][j] == Lost_line)
                    {
                        for(uint8 k = Deal_Bottom; k <= Deal_Top + 1; k++)
                        {
                            if(imgOSTU[k][j] == Black || imgOSTU[k][j] == Left_line || imgOSTU[k][j] == Right_line)
                            {
                                r_num += 1.0f;
                                r_sum += (float)k;
                                break;
                            }
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                break;
            }
        }

        if(l_num > 0.0f && r_num > 0.0f)
        {
            float l_average = l_sum / l_num;
            float r_average = r_sum / r_num;

            if(l_average >= r_average)
            {
                for(int x = r_point; x >= l_point; x--)
                {
                    if(imgOSTU[Lost_Bottom][x] == Lost_line)
                    {
                        imgOSTU[Lost_Bottom][x] = Black;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            else
            {
                for(uint8 x = l_point; x <= r_point; x++)
                {
                    if(imgOSTU[Lost_Bottom][x] == Lost_line)
                    {
                        imgOSTU[Lost_Bottom][x] = Black;
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
        b_lost_num = 1;
    }

    {
        uint32 l_point = 0;
        uint32 r_point = 0;

        for(uint8 x = start_point_l; x < start_point_r; x++)
        {
            if(imgOSTU[Lost_Bottom][x] == Lost_line)
            {
                l_point = x;
                break;
            }
        }
        for(int x = start_point_r; x > start_point_l; x--)
        {
            if(imgOSTU[Lost_Bottom][x] == Lost_line)
            {
                r_point = (uint32)x;
                break;
            }
        }

        start_center_x = (uint8)((l_point + r_point) / 2U);
        start_center = start_center_x;
    }
}

uint8 Get_top_straightline(void)
{
    uint8 black_num = 0;

    if(t_lost_num == 1)
    {
        if((t_lost_tip >= (uint8)(white_width[Deal_Top] + 5.0f) && t_lost_num == 1) ||
           (t_lost_tip >= (uint8)(3.0f * white_width[Deal_Top])))
        {
            uint32 max = 0;
            uint32 min = Deal_Top;
            uint32 mid;
            uint8 white_low[XM] = {0};

            for(uint8 x = Deal_Left; x <= Deal_Right; x++)
            {
                if(imgOSTU[Lost_Top][x] == Lost_line)
                {
                    for(int y = Deal_Top; y >= 0; y--)
                    {
                        if(imgOSTU[y][x] != White)
                        {
                            white_low[x] = (uint8)y;
                            if(max < (uint32)y) max = (uint32)y;
                            if(min > (uint32)y) min = (uint32)y;
                            break;
                        }
                    }
                }
            }

            mid = (max + min) / 2U;
            for(uint8 x = Deal_Left; x <= Deal_Right; x++)
            {
                if(imgOSTU[Lost_Top][x] == Lost_line && white_low[x] > mid)
                {
                    if(t_lost_tip > 0) t_lost_tip--;
                    imgOSTU[Lost_Top][x] = Black;
                }
            }

            {
                uint32 l_point = 0;
                uint32 r_point = 0;

                end_center_y = Deal_Top;
                for(uint8 x = Deal_Left; x <= Deal_Right; x++)
                {
                    if(imgOSTU[Lost_Top][x] == Lost_line)
                    {
                        l_point = x;
                        break;
                    }
                }
                for(int x = Deal_Right; x >= Deal_Left; x--)
                {
                    if(imgOSTU[Lost_Top][x] == Lost_line)
                    {
                        r_point = (uint32)x;
                        break;
                    }
                }
                end_center_x = (uint8)((l_point + r_point) / 2U);
            }
        }
        else
        {
            end_center_y = Deal_Top;
            end_center_x = (uint8)t_center[0];
        }

        if(end_center_y <= start_center_y)
        {
            return 0;
        }

        black_num = draw_control_line(start_center_x, start_center_y, end_center_x, end_center_y, Control_line);
    }
    else if(t_lost_num == 2)
    {
        uint8 left_line[YM];
        uint8 right_line[YM];
        uint8 black_num_l = 0;
        uint8 black_num_r = 0;
        float kl;
        float kr;

        memset(left_line, XM / 2, sizeof(left_line));
        memset(right_line, XM / 2, sizeof(right_line));

        end_center_y = Deal_Top;
        kl = ((float)t_center[0] - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
        kr = ((float)t_center[1] - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);

        for(uint8 y = start_center_y; y <= end_center_y; y++)
        {
            left_line[y] = (uint8)(start_center_x + kl * (float)(y - start_center_y));
            if(imgOSTU[y][left_line[y]] != White)
            {
                black_num_l++;
            }
            else
            {
                imgOSTU[y][left_line[y]] = Judge_line;
            }
        }
        for(uint8 y = start_center_y; y <= end_center_y; y++)
        {
            right_line[y] = (uint8)(start_center_x + kr * (float)(y - start_center_y));
            if(imgOSTU[y][right_line[y]] != White)
            {
                black_num_r++;
            }
            else
            {
                imgOSTU[y][right_line[y]] = Judge_line;
            }
        }

        if(black_num_l > black_num_r)
        {
            end_center_x = (uint8)t_center[1];
            black_num = black_num_r;
            for(uint8 y = start_center_y; y <= end_center_y; y++)
            {
                mid_line[y] = right_line[y];
                imgOSTU[y][mid_line[y]] = Control_line;
            }
            mid_line_valid = 1;
        }
        else
        {
            end_center_x = (uint8)t_center[0];
            black_num = black_num_l;
            for(uint8 y = start_center_y; y <= end_center_y; y++)
            {
                mid_line[y] = left_line[y];
                imgOSTU[y][mid_line[y]] = Control_line;
            }
            mid_line_valid = 1;
        }
    }

    if(start_center_y == Deal_Bottom && black_num < 3)
    {
        IF = straightlineL;
    }
    else if(start_center_y == Deal_Bottom)
    {
        IF = straightlineS;
    }

    return end_center_y;
}

void Get_curve_line(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 n)
{
    (void)n;
    draw_control_line(x1, y1, x2, y2, Control_line);
}

uint8 Left_curve_line(void)
{
    if(l_lost_num == 0)
    {
        return 0;
    }

    end_center_x = Deal_Left;
    end_center_y = (uint8)l_center[l_lost_num - 1];
    if(end_center_y <= start_center_y)
    {
        return 0;
    }

    (void)draw_control_line(start_center_x, start_center_y, end_center_x, end_center_y, Control_line);
    IF = curve;
    return end_center_y;
}

uint8 Right_curve_line(void)
{
    if(r_lost_num == 0)
    {
        return 0;
    }

    end_center_x = Deal_Right;
    end_center_y = (uint8)r_center[r_lost_num - 1];
    if(end_center_y <= start_center_y)
    {
        return 0;
    }

    (void)draw_control_line(start_center_x, start_center_y, end_center_x, end_center_y, Control_line);
    IF = curve;
    return end_center_y;
}

uint8 Get_Annulus(void)
{
    return 0;
}

uint8 Go_Annulus(void)
{
    Annulus_flag = 0;
    return 0;
}

uint8 Deal_crossroads(void)
{
    return 0;
}

uint8 Get_stopline(void)
{
    return 0;
}

uint8 Disappear_detection(void)
{
    return 0;
}

uint8 Deal_disappear(void)
{
    return 0;
}

void ProcessImg_UpdateMotion10ms(int16 speed_l, int16 speed_r)
{
    float avg_count = ((float)abs_i32(speed_l) + (float)abs_i32(speed_r)) * 0.5f;
    Length += avg_count * PROCESS_MM_PER_COUNT;
}
