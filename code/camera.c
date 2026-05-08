#include "camera.h"
#include "zf_device_ips200.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_driver_dma.h"
#include <string.h>

// ========================= 宏定义与开关 =========================
#define ENABLE_DISPLAY 1             // 是否开启屏幕显示 (正赛设为0)
#define LOST_LINE_REPLACE_VAL (MT9V03X_1_W / 2)    // 丢线时的默认替代中点值(如屏幕中心)
#define TRACK_LINE_IS_BLACK 0        // 1=黑线白底, 0=白线黑底
#define CAMERA_CLOSE_THRESHOLD_OFFSET 5
#define CAMERA_MID_THRESHOLD_OFFSET   10
#define CAMERA_FAR_THRESHOLD_OFFSET   25
#define DISPLAY_IMAGE_DIV              2
#define DISPLAY_REFRESH_SKIP           10
#define CAMERA_FAST_FILTER_MIN_COUNT   4
#define CAMERA_COMPONENT_WIDTH_MAX     48
#define CAMERA_COMPONENT_WHITE_MIN     32
#define CAMERA_COMPONENT_SEGMENT_MIN   2
#define CORNER_OFFSET_THRESHOLD        14
#define CORNER_SCORE_THRESHOLD         7
#define CORNER_HOLD_FRAMES             8
#define CORNER_LOST_COUNT_TRIGGER      35
#define CORNER_LOST_SCORE_THRESHOLD    3

// 对外接口变量
// line_mid[] and track_offset are consumed by motor.c in the 10ms control ISR.
// If compiler optimization is enabled later, consider making shared variables volatile.
int16 line_mid[MT9V03X_1_H];         // 赛道中线
int16 track_offset = 0;              // 赛道偏移量
uint8 junction_type_from_camera = 0; // 【新增】路口标志位: 0=正常, 1=T字, 2=十字
static uint8 line_lost_count = 0;
static uint8 corner_type = 0;        // 0=none, 1=left corner, 2=right corner
static uint8 line_found[MT9V03X_1_H];
static uint8 line_width[MT9V03X_1_H];
static uint8 camera_threshold_base = 0;
static uint8 camera_threshold_near = 0;
static uint8 camera_threshold_mid = 0;
static uint8 camera_threshold_far = 0;
static uint8 white_num_row[MT9V03X_1_H];
static uint8 white_num_col[MT9V03X_1_W];
static uint8 line_confidence = 0;
static uint8 component_row_count = 0;
static uint8 row_segment_count[MT9V03X_1_H];

// ========================= 原图与双缓冲 =========================
// raw_snapshot stores one stable grayscale frame copied from the camera DMA buffer.
// binary_buf_0/1 and line_mid_buf_0/1 are swapped so display does not read data
// while the next frame is being processed.
static uint8 raw_snapshot[MT9V03X_1_H][MT9V03X_1_W];
static uint8 binary_buf_0[MT9V03X_1_H][MT9V03X_1_W];
static uint8 binary_buf_1[MT9V03X_1_H][MT9V03X_1_W];

static int16 line_mid_buf_0[MT9V03X_1_H];
static int16 line_mid_buf_1[MT9V03X_1_H];

static uint8 (*process_image)[MT9V03X_1_W] = binary_buf_0;
static int16 *process_line_mid = line_mid_buf_0;

static uint8 (*display_image)[MT9V03X_1_W] = binary_buf_1;
static int16 *display_line_mid = line_mid_buf_1;

static uint8 image_ready = 0;

// ========================= 摄像头初始化 =========================
void cam_init(void)
{
    // Current project only enables camera 1. Do not use camera 2 pins unless
    // mt9v03x_2 or mt9v03x_double is selected and the wiring is checked.
    mt9v03x_double_init(mt9v03x_1);
}

// ========================= 缓冲交换 =========================
static void image_swap_buffer(void)
{
    // After processing one frame, swap process/display buffers in O(1) time.
    uint8 (*temp_image)[MT9V03X_1_W] = process_image;
    process_image = display_image;
    display_image = temp_image;

    int16 *temp_line_mid = process_line_mid;
    process_line_mid = display_line_mid;
    display_line_mid = temp_line_mid;
}

// ========================= 抓取一帧图像 =========================
static void camera_copy_stable_frame(void)
{
    // In single-camera mode, the driver copies camera1 into mt9v03x_image_2 as a stable frame.
    // This prevents the display and vision code from seeing two mixed frames.
    memcpy(raw_snapshot[0], mt9v03x_image_2[0], MT9V03X_1_W * MT9V03X_1_H);
}

// ========================= 大津法求动态阈值 (极限提速版) =========================
static uint8 clamp_threshold_i16(int16 value)
{
    if(value < 1) return 1;
    if(value > 254) return 254;
    return (uint8)value;
}

static uint8 compute_otsu_threshold(void)
{
    // Otsu selects a threshold from the grayscale histogram. It adapts better
    // than a fixed threshold when track lighting changes.
    int histogram[256] = {0};
    int pixel_count = MT9V03X_1_H * MT9V03X_1_W;
    uint8 *img_ptr = &raw_snapshot[0][0];

    // 1. 统计灰度直方图 (使用 1D 指针遍历提速)
    for(int i = 0; i < pixel_count; i++) {
        histogram[img_ptr[i]]++;
    }

    // 2. 计算总灰度值
    int sum = 0;
    for(int i = 0; i < 256; i++) {
        sum += i * histogram[i];
    }

    int sumB = 0, wB = 0, wF = 0;
    float varMax = 0.0;
    uint8 threshold = 0;

    // 3. 寻找最大类间方差 (数学等效化简，消除大量的除法和浮点运算)
    for(int i = 0; i < 256; i++) {
        wB += histogram[i];
        if (wB == 0) continue;

        wF = pixel_count - wB;
        if (wF == 0) break;

        sumB += i * histogram[i];
        int sumF = sum - sumB;

        // 等效化简公式：Var = (sumB^2 / wB) + (sumF^2 / wF)
        float varBetween = (float)sumB * sumB / wB + (float)sumF * sumF / wF;

        if (varBetween > varMax) {
            varMax = varBetween;
            threshold = i;
        }
    }

    // 容错处理
    if(threshold < 30) threshold = 30;
    if(threshold > 220) threshold = 220;

    return threshold;
}

static void camera_update_corner_type(void)
{
    static uint8 hold_count = 0;
    uint8 left_score = 0;
    uint8 right_score = 0;
    int16 center = MT9V03X_1_W / 2;
    uint8 raw_corner = 0;

    for(int y = MT9V03X_1_H / 8; y < MT9V03X_1_H * 3 / 4; y++)
    {
        if(!line_found[y])
        {
            continue;
        }

        if(process_line_mid[y] < center - CORNER_OFFSET_THRESHOLD)
        {
            left_score++;
        }
        else if(process_line_mid[y] > center + CORNER_OFFSET_THRESHOLD)
        {
            right_score++;
        }
    }

    if(left_score >= CORNER_SCORE_THRESHOLD && left_score > right_score + 3)
    {
        raw_corner = 1;
    }
    else if(right_score >= CORNER_SCORE_THRESHOLD && right_score > left_score + 3)
    {
        raw_corner = 2;
    }
    else if(line_lost_count >= CORNER_LOST_COUNT_TRIGGER)
    {
        if(left_score >= CORNER_LOST_SCORE_THRESHOLD && left_score > right_score)
        {
            raw_corner = 1;
        }
        else if(right_score >= CORNER_LOST_SCORE_THRESHOLD && right_score > left_score)
        {
            raw_corner = 2;
        }
    }

    if(raw_corner != 0)
    {
        corner_type = raw_corner;
        hold_count = CORNER_HOLD_FRAMES;
    }
    else if(hold_count > 0)
    {
        hold_count--;
    }
    else
    {
        corner_type = 0;
    }
}

// ========================= 图像处理任务 =========================
void image_process_task(void)
{
    if (mt9v03x_finish_flag_1 == 1)
    {
        // Driver copies a stable frame before DMA restarts.
        // Clear the finish flag after taking that stable snapshot.
        camera_copy_stable_frame();
        mt9v03x_finish_flag_1 = 0;

        // 2. 计算动态阈值并二值化 (使用 1D 指针大幅提升效率)
        uint8 dynamic_threshold = compute_otsu_threshold();
        camera_threshold_base = dynamic_threshold;
        camera_threshold_near = clamp_threshold_i16((int16)dynamic_threshold + CAMERA_CLOSE_THRESHOLD_OFFSET);
        camera_threshold_mid  = clamp_threshold_i16((int16)dynamic_threshold + CAMERA_MID_THRESHOLD_OFFSET);
        camera_threshold_far  = clamp_threshold_i16((int16)dynamic_threshold + CAMERA_FAR_THRESHOLD_OFFSET);
        memset(white_num_row, 0, sizeof(white_num_row));
        memset(white_num_col, 0, sizeof(white_num_col));
        memset(line_found, 0, sizeof(line_found));
        memset(line_width, 0, sizeof(line_width));
        memset(row_segment_count, 0, sizeof(row_segment_count));
        component_row_count = 0;

        for(int y = 0; y < MT9V03X_1_H; y++)
        {
            uint8 row_threshold;

            if(y < MT9V03X_1_H / 3)
            {
                row_threshold = camera_threshold_far;
            }
            else if(y < MT9V03X_1_H * 2 / 3)
            {
                row_threshold = camera_threshold_mid;
            }
            else
            {
                row_threshold = camera_threshold_near;
            }

            for(int x = 0; x < MT9V03X_1_W; x++)
            {
                uint8 neighbor_count = 0;

#if TRACK_LINE_IS_BLACK
                if(raw_snapshot[y][x] < row_threshold) neighbor_count++;
                if((x > 0) && (raw_snapshot[y][x - 1] < row_threshold)) neighbor_count++;
                if((x < MT9V03X_1_W - 1) && (raw_snapshot[y][x + 1] < row_threshold)) neighbor_count++;
                if((y > 0) && (raw_snapshot[y - 1][x] < row_threshold)) neighbor_count++;
                if((y < MT9V03X_1_H - 1) && (raw_snapshot[y + 1][x] < row_threshold)) neighbor_count++;
#else
                if(raw_snapshot[y][x] > row_threshold) neighbor_count++;
                if((x > 0) && (raw_snapshot[y][x - 1] > row_threshold)) neighbor_count++;
                if((x < MT9V03X_1_W - 1) && (raw_snapshot[y][x + 1] > row_threshold)) neighbor_count++;
                if((y > 0) && (raw_snapshot[y - 1][x] > row_threshold)) neighbor_count++;
                if((y < MT9V03X_1_H - 1) && (raw_snapshot[y + 1][x] > row_threshold)) neighbor_count++;
#endif

                uint8 valid_pixel = (neighbor_count >= CAMERA_FAST_FILTER_MIN_COUNT);
                process_image[y][x] = valid_pixel ? 255 : 0;
                if(valid_pixel)
                {
                    white_num_row[y]++;
                    white_num_col[x]++;
                }
            }
        }

        // 3. 防噪点扫线法
        int lost_line_count = 0; // 记录丢线的行数(用于简单的路口检测)

        for (int i = MT9V03X_1_H - 1; i >= 0; i--)
        {
            // Search starts from the previous row center. This makes the scan
            // faster and helps reject isolated noise far away from the lane.
            int center_seed = (i == MT9V03X_1_H - 1) ? (MT9V03X_1_W / 2) : process_line_mid[i + 1];

            if (center_seed < 0) center_seed = 0;
            if (center_seed >= MT9V03X_1_W) center_seed = MT9V03X_1_W - 1;

            int line_pos = -1;
            int left;
            int right;
            uint8 segment_count = 0;
            uint8 in_segment = 0;

            for(int x = 0; x < MT9V03X_1_W; x++)
            {
                if(process_image[i][x] != 0)
                {
                    if(!in_segment)
                    {
                        segment_count++;
                        in_segment = 1;
                    }
                }
                else
                {
                    in_segment = 0;
                }
            }
            row_segment_count[i] = segment_count;

            for (int span = 0; span < MT9V03X_1_W / 2; span++)
            {
                left = center_seed - span;
                right = center_seed + span;

                if ((left >= 0) && (process_image[i][left] != 0))
                {
                    line_pos = left;
                    break;
                }
                if ((right < MT9V03X_1_W) && (process_image[i][right] != 0))
                {
                    line_pos = right;
                    break;
                }
            }

            if (line_pos < 0)
            {
                process_line_mid[i] = center_seed;
                line_found[i] = 0;
                line_width[i] = 0;
                lost_line_count++;
                continue;
            }

            left = line_pos;
            right = line_pos;
            while ((left > 0) && (process_image[i][left] != 0)) left--;
            while ((right < MT9V03X_1_W - 1) && (process_image[i][right] != 0)) right++;

            int width = right - left;

            // 【核心优化】：丢线与元件区检测
            if (width < 3)
            {
                process_line_mid[i] = center_seed;
                line_found[i] = 0;
                line_width[i] = 0;
                lost_line_count++;
            }
            else if(width > CAMERA_COMPONENT_WIDTH_MAX ||
                    (row_segment_count[i] >= CAMERA_COMPONENT_SEGMENT_MIN &&
                     white_num_row[i] >= CAMERA_COMPONENT_WHITE_MIN))
            {
                // LED/元件图案会形成异常宽的白色连通区，不参与普通中线计算。
                process_line_mid[i] = center_seed;
                line_found[i] = 0;
                line_width[i] = (uint8)width;
                component_row_count++;
            }
            else
            {
                process_line_mid[i] = (int16)((left + right) / 2);
                line_found[i] = 1;
                line_width[i] = (uint8)width;
            }
        }

        // 【新增】：极简的路口判断逻辑演示
        // 如果底部和中部大量丢线或者赛道宽度突变，这里可以赋值给电机控制层
        // 实际比赛中需要结合L角检测、双边丢线等更复杂的特征
        if(lost_line_count > (MT9V03X_1_H / 2))
        {
            // junction_type_from_camera = 1; // 触发后由电机层识别并清零
        }
        line_lost_count = (lost_line_count > 255) ? 255 : (uint8)lost_line_count;
        line_confidence = (uint8)((MT9V03X_1_H - line_lost_count) * 100 / MT9V03X_1_H);
        camera_update_corner_type();

        // 4. 计算前瞻偏移量 (加权平均法)
        int32 sum = 0;
        int32 weight_sum = 0;
        int start = MT9V03X_1_H / 3;
        int end   = MT9V03X_1_H * 5 / 6;

        for (int i = start; i < end; i++)
        {
            if(!line_found[i])
            {
                continue;
            }
            if(row_segment_count[i] >= CAMERA_COMPONENT_SEGMENT_MIN && white_num_row[i] >= CAMERA_COMPONENT_WHITE_MIN)
            {
                continue;
            }

            // track_offset > 0 means the detected lane center is to the right
            // of the image center; motor.c converts it into differential speed.
            int weight = i;
            sum += (process_line_mid[i] - (MT9V03X_1_W / 2)) * weight;
            weight_sum += weight;
        }
        track_offset = (weight_sum > 0) ? (int16)(sum / weight_sum) : 0;

        // 5. 同步中线给外部循迹代码
        for (int i = 0; i < MT9V03X_1_H; i++)
        {
            line_mid[i] = process_line_mid[i];
        }

        // 6. 交换显示缓冲
        image_swap_buffer();
        image_ready = 1;
    }
}

// ========================= 图像显示任务 =========================
void image_display_task(void)
{
#if ENABLE_DISPLAY
    static uint8 refresh_cnt = 0;
    if (!image_ready) return;

    // IPS200 refresh is slow compared with image processing, so show a scaled,
    // low-rate preview. Set ENABLE_DISPLAY to 0 for race runs.
    if (++refresh_cnt < DISPLAY_REFRESH_SKIP) return;
    refresh_cnt = 0;

    ips200_show_gray_image(0, 0, display_image[0], MT9V03X_1_W, MT9V03X_1_H, MT9V03X_1_W / DISPLAY_IMAGE_DIV, MT9V03X_1_H / DISPLAY_IMAGE_DIV, 0);

    for (int i = 0; i < MT9V03X_1_H; i += DISPLAY_IMAGE_DIV)
    {
        if ((display_line_mid[i] >= 0) && (display_line_mid[i] < MT9V03X_1_W))
        {
            ips200_draw_point((uint16)(display_line_mid[i] / DISPLAY_IMAGE_DIV), (uint16)(i / DISPLAY_IMAGE_DIV), RGB565_RED);
        }
    }
#endif
}

int16 Camera_GetTrackOffset(void)
{
    return track_offset;
}

uint8 Camera_GetLineLostCount(void)
{
    return line_lost_count;
}

uint8 Camera_GetCornerType(void)
{
    return corner_type;
}

uint8 Camera_IsFrameReady(void)
{
    return image_ready;
}

uint8 Camera_GetLineConfidence(void)
{
    return line_confidence;
}

uint8 Camera_GetComponentRowCount(void)
{
    return component_row_count;
}

uint8 Camera_GetThreshold(void)
{
    return camera_threshold_base;
}

uint8 Camera_GetThresholdNear(void)
{
    return camera_threshold_near;
}

uint8 Camera_GetThresholdMid(void)
{
    return camera_threshold_mid;
}

uint8 Camera_GetThresholdFar(void)
{
    return camera_threshold_far;
}

uint8 Camera_GetWhiteRow(uint8 row)
{
    if(row >= MT9V03X_1_H) return 0;
    return white_num_row[row];
}

uint8 Camera_GetWhiteCol(uint8 col)
{
    if(col >= MT9V03X_1_W) return 0;
    return white_num_col[col];
}

int16 Camera_GetCenterLine(uint8 row)
{
    if(row >= MT9V03X_1_H)
    {
        return MT9V03X_1_W / 2;
    }
    return line_mid[row];
}
