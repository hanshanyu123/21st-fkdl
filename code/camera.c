#include "camera.h"
#include "zf_device_ips200.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_driver_dma.h"
#include <string.h>

#define ENABLE_DISPLAY                 1
#define LOST_LINE_REPLACE_VAL          (MT9V03X_1_W / 2)
#define TRACK_LINE_IS_BLACK            0

#define CAMERA_CLOSE_THRESHOLD_OFFSET  5
#define CAMERA_MID_THRESHOLD_OFFSET    10
#define CAMERA_FAR_THRESHOLD_OFFSET    25

#define DISPLAY_IMAGE_DIV              2
#define DISPLAY_REFRESH_SKIP           10

#define CAMERA_FAST_FILTER_MIN_COUNT   4
#define CAMERA_ROW_SEGMENT_MAX         8
#define CAMERA_SEGMENT_MIN_WIDTH       3
#define CAMERA_SEGMENT_CONNECT_MARGIN  14
#define CAMERA_SEGMENT_CENTER_JUMP_MAX 38
#define CAMERA_START_SEARCH_ROWS       36
#define CAMERA_START_CENTER_LIMIT      (MT9V03X_1_W / 3)

#define CAMERA_EDGE_LOST_MARGIN        5
#define CAMERA_BOTTOM_START_ROW        (MT9V03X_1_H * 2 / 3)
#define CAMERA_TOP_START_ROW           (MT9V03X_1_H / 8)
#define CAMERA_TOP_END_ROW             (MT9V03X_1_H / 3)

#define CAMERA_COMPONENT_ROWS_MIN      5
#define CAMERA_COMPONENT_SEGMENT_MIN   2
#define CAMERA_COMPONENT_WHITE_MIN     32
#define CAMERA_COMPONENT_WIDTH_MAX     48
#define CAMERA_COMPONENT_WIDTH_EXTRA   12
#define CAMERA_COMPONENT_ENTER_FRAMES  2
#define CAMERA_COMPONENT_LOCK_MIN      8
#define CAMERA_COMPONENT_EXIT_FRAMES   5
#define CAMERA_CORNER_COMPONENT_BLOCK  10

#define CAMERA_CORNER_SIDE_BLOCK_MIN   1
#define CAMERA_CORNER_SIDE_LOST_MIN    8
#define CAMERA_CORNER_SIDE_DOMINANCE   5
#define CAMERA_CORNER_OPPOSITE_MAX     2
#define CAMERA_CORNER_TOP_LOST_MIN     12
#define CAMERA_CORNER_BOTTOM_VALID_MIN 8
#define CAMERA_CORNER_EARLY_SIDE_LOST_MIN  6
#define CAMERA_CORNER_EARLY_TOP_LOST_MAX   8
#define CAMERA_CORNER_ENTER_FRAMES     2
#define CAMERA_CORNER_TURN_MIN_FRAMES  8
#define CAMERA_CORNER_EXIT_FRAMES      5
#define CAMERA_CORNER_EXIT_VALID_MIN   28
#define CAMERA_LOST_CONFIRM_FRAMES     3
#define CAMERA_PATCH_QUALITY_MIN       35
#define CAMERA_PATCH_BLACK_MAX_PERCENT 75
#define CAMERA_CORNER_PATCH_END_ROW    (MT9V03X_1_H / 3)

#define CAMERA_LOST_BLOCK_MAX          3
#define CAMERA_LOST_BLOCK_MIN_LEN      3

#define CAMERA_WIDTH_TABLE_COUNT       13
#define CAMERA_EDGE_MIGRATE_EXTRA      4
#define CAMERA_EDGE_MIGRATE_WHITE_MIN  4
#define CAMERA_WIDE_SPLIT_EXTRA        10
#define CAMERA_WIDE_SPLIT_RUN_DELTA    4

#define CAMERA_OFFSET_STEP_MAX         8
#define CAMERA_CORNER_OFFSET_STEP_MAX  14
#define CAMERA_COMPONENT_OFFSET_STEP   4

int16 line_mid[MT9V03X_1_H];
int16 track_offset = 0;
uint8 junction_type_from_camera = 0;

static uint8 line_lost_count = 0;
static uint8 corner_type = 0;
static uint8 vision_state = VISION_NORMAL;
static uint8 vision_phase = VISION_PHASE_NORMAL;
static uint8 corner_direction = 0;
static uint8 line_confidence = 0;
static uint8 component_row_count = 0;
static uint8 corner_patch_quality = 0;

static uint8 left_lost_count = 0;
static uint8 right_lost_count = 0;
static uint8 top_lost_count = 0;
static uint8 bottom_lost_count = 0;
static uint8 left_lost_num = 0;
static uint8 right_lost_num = 0;
static uint8 top_lost_num = 0;
static uint8 bottom_lost_num = 0;
static uint8 left_lost_center[CAMERA_LOST_BLOCK_MAX];
static uint8 right_lost_center[CAMERA_LOST_BLOCK_MAX];
static uint8 top_lost_center[CAMERA_LOST_BLOCK_MAX];
static uint8 bottom_lost_center[CAMERA_LOST_BLOCK_MAX];

static uint8 camera_threshold_base = 0;
static uint8 camera_threshold_near = 0;
static uint8 camera_threshold_mid = 0;
static uint8 camera_threshold_far = 0;

static uint8 white_num_row[MT9V03X_1_H];
static uint8 white_num_col[MT9V03X_1_W];
static uint8 row_segment_count[MT9V03X_1_H];
static uint8 line_found[MT9V03X_1_H];
static uint8 line_width[MT9V03X_1_H];
static uint8 component_row[MT9V03X_1_H];
static uint8 left_touch_edge[MT9V03X_1_H];
static uint8 right_touch_edge[MT9V03X_1_H];
static int16 left_boundary[MT9V03X_1_H];
static int16 right_boundary[MT9V03X_1_H];

static uint8 segment_start[MT9V03X_1_H][CAMERA_ROW_SEGMENT_MAX];
static uint8 segment_end[MT9V03X_1_H][CAMERA_ROW_SEGMENT_MAX];

static const uint8 camera_width_table[CAMERA_WIDTH_TABLE_COUNT] =
{
    6, 6, 7, 8, 10, 12, 15, 19, 24, 30, 36, 42, 46
};

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

void cam_init(void)
{
    mt9v03x_double_init(mt9v03x_1);
}

static void image_swap_buffer(void)
{
    uint8 (*temp_image)[MT9V03X_1_W] = process_image;
    process_image = display_image;
    display_image = temp_image;

    int16 *temp_line_mid = process_line_mid;
    process_line_mid = display_line_mid;
    display_line_mid = temp_line_mid;
}

static void camera_copy_stable_frame(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    memcpy(raw_snapshot[0], mt9v03x_image_2[0], MT9V03X_1_W * MT9V03X_1_H);
    interrupt_global_enable(interrupt_state);
}

static uint8 clamp_threshold_i16(int16 value)
{
    if(value < 1) return 1;
    if(value > 254) return 254;
    return (uint8)value;
}

static int16 clamp_image_x(int32 value)
{
    if(value < 0) return 0;
    if(value >= MT9V03X_1_W) return MT9V03X_1_W - 1;
    return (int16)value;
}

static uint8 compute_otsu_threshold(void)
{
    int histogram[256] = {0};
    int pixel_count = MT9V03X_1_H * MT9V03X_1_W;
    uint8 *img_ptr = &raw_snapshot[0][0];
    int sum = 0;
    int sumB = 0;
    int wB = 0;
    int wF = 0;
    float varMax = 0.0f;
    uint8 threshold = 0;

    for(int i = 0; i < pixel_count; i++)
    {
        histogram[img_ptr[i]]++;
    }

    for(int i = 0; i < 256; i++)
    {
        sum += i * histogram[i];
    }

    for(int i = 0; i < 256; i++)
    {
        wB += histogram[i];
        if(wB == 0) continue;

        wF = pixel_count - wB;
        if(wF == 0) break;

        sumB += i * histogram[i];
        int sumF = sum - sumB;
        float varBetween = (float)sumB * sumB / wB + (float)sumF * sumF / wF;

        if(varBetween > varMax)
        {
            varMax = varBetween;
            threshold = (uint8)i;
        }
    }

    if(threshold < 30) threshold = 30;
    if(threshold > 220) threshold = 220;
    return threshold;
}

static uint8 camera_expected_width(uint8 row)
{
    int anchor_pos = (int)row * (CAMERA_WIDTH_TABLE_COUNT - 1);
    int base = anchor_pos / (MT9V03X_1_H - 1);
    int rem = anchor_pos % (MT9V03X_1_H - 1);
    int32 width;

    if(base >= CAMERA_WIDTH_TABLE_COUNT - 1)
    {
        width = camera_width_table[CAMERA_WIDTH_TABLE_COUNT - 1];
    }
    else
    {
        width = camera_width_table[base] +
                ((int32)(camera_width_table[base + 1] - camera_width_table[base]) * rem) /
                (MT9V03X_1_H - 1);
    }

    if(width < 3) width = 3;
    if(width > MT9V03X_1_W) width = MT9V03X_1_W;
    return (uint8)width;
}

static void camera_add_segment(uint8 row, uint8 *count, int start, int end)
{
    if(start < 0) start = 0;
    if(end >= MT9V03X_1_W) end = MT9V03X_1_W - 1;
    if(end < start) return;
    if((end - start + 1) < CAMERA_SEGMENT_MIN_WIDTH) return;

    if(*count < CAMERA_ROW_SEGMENT_MAX)
    {
        segment_start[row][*count] = (uint8)start;
        segment_end[row][*count] = (uint8)end;
    }
    (*count)++;
}

static uint8 camera_row_is_component(uint8 row, uint8 selected_width)
{
    uint8 expected_width = camera_expected_width(row);
    uint8 multi_segment_white_min = expected_width * 2;
    uint16 width_limit = (uint16)expected_width * 2 + CAMERA_COMPONENT_WIDTH_EXTRA;

    if(multi_segment_white_min < CAMERA_COMPONENT_WHITE_MIN)
    {
        multi_segment_white_min = CAMERA_COMPONENT_WHITE_MIN;
    }
    if(width_limit < CAMERA_COMPONENT_WIDTH_MAX)
    {
        width_limit = CAMERA_COMPONENT_WIDTH_MAX;
    }

    if(row_segment_count[row] >= CAMERA_COMPONENT_SEGMENT_MIN &&
       white_num_row[row] >= multi_segment_white_min)
    {
        return 1;
    }

    if(row >= (MT9V03X_1_H / 3) && selected_width > width_limit)
    {
        return 1;
    }

    return 0;
}

static void camera_reset_frame_features(void)
{
    memset(white_num_row, 0, sizeof(white_num_row));
    memset(white_num_col, 0, sizeof(white_num_col));
    memset(row_segment_count, 0, sizeof(row_segment_count));
    memset(line_found, 0, sizeof(line_found));
    memset(line_width, 0, sizeof(line_width));
    memset(component_row, 0, sizeof(component_row));
    memset(left_touch_edge, 0, sizeof(left_touch_edge));
    memset(right_touch_edge, 0, sizeof(right_touch_edge));
    memset(segment_start, 0, sizeof(segment_start));
    memset(segment_end, 0, sizeof(segment_end));
    component_row_count = 0;
    corner_patch_quality = 0;

    for(int y = 0; y < MT9V03X_1_H; y++)
    {
        process_line_mid[y] = LOST_LINE_REPLACE_VAL;
        left_boundary[y] = 0;
        right_boundary[y] = MT9V03X_1_W - 1;
    }
}

static void camera_binarize_frame(void)
{
    uint8 dynamic_threshold = compute_otsu_threshold();

    camera_threshold_base = dynamic_threshold;
    camera_threshold_near = clamp_threshold_i16((int16)dynamic_threshold + CAMERA_CLOSE_THRESHOLD_OFFSET);
    camera_threshold_mid  = clamp_threshold_i16((int16)dynamic_threshold + CAMERA_MID_THRESHOLD_OFFSET);
    camera_threshold_far  = clamp_threshold_i16((int16)dynamic_threshold + CAMERA_FAR_THRESHOLD_OFFSET);

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

            if(neighbor_count >= CAMERA_FAST_FILTER_MIN_COUNT)
            {
                process_image[y][x] = 255;
                white_num_row[y]++;
                white_num_col[x]++;
            }
            else
            {
                process_image[y][x] = 0;
            }
        }
    }
}

static void camera_extract_row_segments(void)
{
    for(int y = 0; y < MT9V03X_1_H; y++)
    {
        uint8 count = 0;
        int x = 0;

        while(x < MT9V03X_1_W)
        {
            while(x < MT9V03X_1_W && process_image[y][x] == 0)
            {
                x++;
            }
            if(x >= MT9V03X_1_W)
            {
                break;
            }

            int start = x;
            while(x < MT9V03X_1_W && process_image[y][x] != 0)
            {
                x++;
            }
            int end = x - 1;

            if((end - start + 1) >= CAMERA_SEGMENT_MIN_WIDTH)
            {
                camera_add_segment((uint8)y, &count, start, end);
            }
        }

        row_segment_count[y] = count;
    }
}

static uint8 camera_column_white_run(uint8 row, int x)
{
    uint8 run = 0;

    if(row >= CAMERA_BOTTOM_START_ROW)
    {
        for(int y = row; y >= 0 && process_image[y][x] != 0 && run < MT9V03X_1_H; y--)
        {
            run++;
        }
    }
    else
    {
        for(int y = row; y < MT9V03X_1_H && process_image[y][x] != 0 && run < MT9V03X_1_H; y++)
        {
            run++;
        }
    }

    return run;
}

static uint8 camera_add_split_candidate(uint8 starts[],
                                        uint8 ends[],
                                        uint8 *count,
                                        int start,
                                        int end)
{
    if(start < 0) start = 0;
    if(end >= MT9V03X_1_W) end = MT9V03X_1_W - 1;
    if(end < start) return 0;
    if((end - start + 1) < CAMERA_SEGMENT_MIN_WIDTH) return 0;

    for(uint8 i = 0; i < *count && i < CAMERA_ROW_SEGMENT_MAX; i++)
    {
        if(starts[i] == start && ends[i] == end)
        {
            return 0;
        }
    }

    if(*count < CAMERA_ROW_SEGMENT_MAX)
    {
        starts[*count] = (uint8)start;
        ends[*count] = (uint8)end;
    }
    (*count)++;
    return 1;
}

static void camera_split_wide_row_segments(uint8 row)
{
    uint8 old_start[CAMERA_ROW_SEGMENT_MAX];
    uint8 old_end[CAMERA_ROW_SEGMENT_MAX];
    uint8 new_start[CAMERA_ROW_SEGMENT_MAX];
    uint8 new_end[CAMERA_ROW_SEGMENT_MAX];
    uint8 old_count = row_segment_count[row];
    uint8 new_count = 0;
    uint8 expected_width = camera_expected_width(row);
    uint8 in_split_zone = (row < CAMERA_TOP_END_ROW || row >= CAMERA_BOTTOM_START_ROW);

    if(old_count > CAMERA_ROW_SEGMENT_MAX)
    {
        old_count = CAMERA_ROW_SEGMENT_MAX;
    }

    for(uint8 i = 0; i < old_count; i++)
    {
        old_start[i] = segment_start[row][i];
        old_end[i] = segment_end[row][i];
    }

    for(uint8 i = 0; i < old_count; i++)
    {
        int start = old_start[i];
        int end = old_end[i];
        int width = end - start + 1;
        int wide_limit = (int)expected_width * 2 + CAMERA_WIDE_SPLIT_EXTRA;
        uint8 added = 0;

        if(!in_split_zone || width <= wide_limit)
        {
            camera_add_split_candidate(new_start, new_end, &new_count, start, end);
            continue;
        }

        {
            uint8 min_run = 255;
            uint8 max_run = 0;

            for(int x = start; x <= end; x++)
            {
                uint8 run = camera_column_white_run(row, x);
                if(run < min_run) min_run = run;
                if(run > max_run) max_run = run;
            }

            if(max_run >= min_run + CAMERA_WIDE_SPLIT_RUN_DELTA)
            {
                uint8 threshold = (uint8)((min_run + max_run) / 2);
                int cluster_start = -1;

                for(int x = start; x <= end; x++)
                {
                    if(camera_column_white_run(row, x) >= threshold)
                    {
                        if(cluster_start < 0) cluster_start = x;
                    }
                    else if(cluster_start >= 0)
                    {
                        added += camera_add_split_candidate(new_start, new_end, &new_count,
                                                            cluster_start, x - 1);
                        cluster_start = -1;
                    }
                }

                if(cluster_start >= 0)
                {
                    added += camera_add_split_candidate(new_start, new_end, &new_count,
                                                        cluster_start, end);
                }
            }
        }

        if(added == 0)
        {
            camera_add_split_candidate(new_start, new_end, &new_count, start, end);
        }
    }

    row_segment_count[row] = new_count;
    for(uint8 i = 0; i < new_count && i < CAMERA_ROW_SEGMENT_MAX; i++)
    {
        segment_start[row][i] = new_start[i];
        segment_end[row][i] = new_end[i];
    }
}

static void camera_split_wide_segments(void)
{
    for(int y = 0; y < MT9V03X_1_H; y++)
    {
        camera_split_wide_row_segments((uint8)y);
    }
}

static int camera_segment_center(uint8 row, uint8 index)
{
    return ((int)segment_start[row][index] + (int)segment_end[row][index]) / 2;
}

static int camera_segment_width(uint8 row, uint8 index)
{
    return (int)segment_end[row][index] - (int)segment_start[row][index] + 1;
}

static int camera_abs_i32(int value)
{
    return (value < 0) ? -value : value;
}

static int camera_find_start_segment(int *start_y, uint8 *start_index)
{
    int best_score = 32767;
    int bottom_limit = MT9V03X_1_H - CAMERA_START_SEARCH_ROWS;

    if(bottom_limit < 0) bottom_limit = 0;

    for(int y = MT9V03X_1_H - 1; y >= bottom_limit; y--)
    {
        uint8 count = row_segment_count[y];
        if(count > CAMERA_ROW_SEGMENT_MAX) count = CAMERA_ROW_SEGMENT_MAX;

        for(uint8 i = 0; i < count; i++)
        {
            int center = camera_segment_center((uint8)y, i);
            int width = camera_segment_width((uint8)y, i);
            int center_error = camera_abs_i32(center - MT9V03X_1_W / 2);
            int width_error = camera_abs_i32(width - camera_expected_width((uint8)y));
            int score = center_error * 3 + width_error;

            if(center_error > CAMERA_START_CENTER_LIMIT)
            {
                continue;
            }
            if(camera_row_is_component((uint8)y, (uint8)width))
            {
                score += 40;
            }
            if(score < best_score)
            {
                best_score = score;
                *start_y = y;
                *start_index = i;
            }
        }
    }

    return best_score < 32767;
}

static int camera_score_connected_segment(uint8 row,
                                          uint8 index,
                                          int prev_left,
                                          int prev_right,
                                          int prev_center)
{
    int left = segment_start[row][index];
    int right = segment_end[row][index];
    int center = (left + right) / 2;
    int width = right - left + 1;
    int expected_width = camera_expected_width(row);
    int expanded_left = prev_left - CAMERA_SEGMENT_CONNECT_MARGIN;
    int expanded_right = prev_right + CAMERA_SEGMENT_CONNECT_MARGIN;
    int overlap_left = (left > expanded_left) ? left : expanded_left;
    int overlap_right = (right < expanded_right) ? right : expanded_right;
    int center_jump = camera_abs_i32(center - prev_center);
    int score;

    if(overlap_right < overlap_left && center_jump > CAMERA_SEGMENT_CENTER_JUMP_MAX)
    {
        return 32767;
    }

    score = center_jump * 3 + camera_abs_i32(width - expected_width);
    if(overlap_right >= overlap_left)
    {
        score -= 20;
    }
    if(camera_row_is_component(row, (uint8)width))
    {
        score += 25;
    }

    return score;
}

static void camera_trace_connected_track(void)
{
    int start_y = 0;
    uint8 start_index = 0;
    int prev_left;
    int prev_right;
    int prev_center;
    int lost_line_count = 0;

    if(!camera_find_start_segment(&start_y, &start_index))
    {
        line_lost_count = MT9V03X_1_H;
        line_confidence = 0;
        return;
    }

    prev_left = segment_start[start_y][start_index];
    prev_right = segment_end[start_y][start_index];
    prev_center = (prev_left + prev_right) / 2;

    for(int y = MT9V03X_1_H - 1; y > start_y; y--)
    {
        process_line_mid[y] = (int16)prev_center;
        left_boundary[y] = (int16)prev_left;
        right_boundary[y] = (int16)prev_right;
        line_found[y] = 1;
        line_width[y] = (uint8)(prev_right - prev_left + 1);
    }

    for(int y = start_y; y >= 0; y--)
    {
        int best_score = 32767;
        int best_index = -1;
        uint8 count = row_segment_count[y];

        if(count > CAMERA_ROW_SEGMENT_MAX) count = CAMERA_ROW_SEGMENT_MAX;

        for(uint8 i = 0; i < count; i++)
        {
            int score = camera_score_connected_segment((uint8)y, i, prev_left, prev_right, prev_center);
            if(score < best_score)
            {
                best_score = score;
                best_index = i;
            }
        }

        if(best_index >= 0 && best_score < 32767)
        {
            int width;
            prev_left = segment_start[y][best_index];
            prev_right = segment_end[y][best_index];
            prev_center = (prev_left + prev_right) / 2;
            width = prev_right - prev_left + 1;

            left_boundary[y] = (int16)prev_left;
            right_boundary[y] = (int16)prev_right;
            process_line_mid[y] = (int16)prev_center;
            line_found[y] = 1;
            line_width[y] = (uint8)((width > 255) ? 255 : width);

            if(camera_row_is_component((uint8)y, line_width[y]))
            {
                component_row[y] = 1;
                component_row_count++;
            }
        }
        else
        {
            process_line_mid[y] = (int16)prev_center;
            left_boundary[y] = (int16)prev_left;
            right_boundary[y] = (int16)prev_right;
            line_found[y] = 0;
            line_width[y] = 0;
            lost_line_count++;
        }
    }

    line_lost_count = (lost_line_count > 255) ? 255 : (uint8)lost_line_count;
    line_confidence = (uint8)((MT9V03X_1_H - line_lost_count) * 100 / MT9V03X_1_H);
}

static uint8 count_valid_rows(int start, int end)
{
    uint8 count = 0;

    if(start < 0) start = 0;
    if(end > MT9V03X_1_H) end = MT9V03X_1_H;

    for(int y = start; y < end; y++)
    {
        if(line_found[y] && !component_row[y])
        {
            count++;
        }
    }

    return count;
}

static void camera_store_lost_block(uint8 *block_num, uint8 centers[], int start, int end)
{
    if((end - start + 1) < CAMERA_LOST_BLOCK_MIN_LEN)
    {
        return;
    }

    if(*block_num < CAMERA_LOST_BLOCK_MAX)
    {
        centers[*block_num] = (uint8)((start + end) / 2);
    }
    (*block_num)++;
}

static void camera_count_lost_blocks_vertical(uint8 *count,
                                              uint8 *block_num,
                                              uint8 centers[],
                                              int start,
                                              int end,
                                              uint8 side)
{
    uint8 in_block = 0;
    int block_start = 0;

    for(int y = start; y < end; y++)
    {
        uint8 lost = 0;

        if(side == 0)
        {
            lost = (!line_found[y] && !component_row[y]);
        }
        else if(side == 1)
        {
            lost = (!component_row[y] && left_touch_edge[y]);
        }
        else
        {
            lost = (!component_row[y] && right_touch_edge[y]);
        }

        if(lost)
        {
            (*count)++;
            if(!in_block)
            {
                in_block = 1;
                block_start = y;
            }
        }
        else if(in_block)
        {
            camera_store_lost_block(block_num, centers, block_start, y - 1);
            in_block = 0;
        }
    }

    if(in_block)
    {
        camera_store_lost_block(block_num, centers, block_start, end - 1);
    }
}

static uint8 camera_edge_white_count(uint8 row, uint8 left_side)
{
    uint8 count = 0;
    int scan_width = camera_expected_width(row) + CAMERA_EDGE_MIGRATE_EXTRA;

    if(scan_width > MT9V03X_1_W / 3)
    {
        scan_width = MT9V03X_1_W / 3;
    }

    if(left_side)
    {
        for(int x = 0; x < scan_width; x++)
        {
            if(process_image[row][x] != 0)
            {
                count++;
            }
        }
    }
    else
    {
        for(int x = MT9V03X_1_W - scan_width; x < MT9V03X_1_W; x++)
        {
            if(process_image[row][x] != 0)
            {
                count++;
            }
        }
    }

    return count;
}

static void camera_update_edge_migration(void)
{
    memset(left_touch_edge, 0, sizeof(left_touch_edge));
    memset(right_touch_edge, 0, sizeof(right_touch_edge));

    for(int y = 0; y < MT9V03X_1_H; y++)
    {
        int edge_margin = camera_expected_width((uint8)y) / 2 + CAMERA_EDGE_LOST_MARGIN;
        int migrate_white_min = camera_expected_width((uint8)y) / 3;
        int center = process_line_mid[y];

        if(migrate_white_min < CAMERA_EDGE_MIGRATE_WHITE_MIN)
        {
            migrate_white_min = CAMERA_EDGE_MIGRATE_WHITE_MIN;
        }

        if(!line_found[y] || component_row[y])
        {
            if(!line_found[y] && !component_row[y])
            {
                if(left_boundary[y] <= edge_margin)
                {
                    left_touch_edge[y] = 1;
                }
                if(right_boundary[y] >= (MT9V03X_1_W - 1 - edge_margin))
                {
                    right_touch_edge[y] = 1;
                }
            }
            continue;
        }

        if(left_boundary[y] <= edge_margin ||
           (center < MT9V03X_1_W / 3 &&
            camera_edge_white_count((uint8)y, 1) >= migrate_white_min))
        {
            left_touch_edge[y] = 1;
        }

        if(right_boundary[y] >= (MT9V03X_1_W - 1 - edge_margin) ||
           (center > MT9V03X_1_W * 2 / 3 &&
            camera_edge_white_count((uint8)y, 0) >= migrate_white_min))
        {
            right_touch_edge[y] = 1;
        }
    }
}

static void camera_update_lost_features(void)
{
    left_lost_count = 0;
    right_lost_count = 0;
    top_lost_count = 0;
    bottom_lost_count = 0;
    left_lost_num = 0;
    right_lost_num = 0;
    top_lost_num = 0;
    bottom_lost_num = 0;
    memset(left_lost_center, 0, sizeof(left_lost_center));
    memset(right_lost_center, 0, sizeof(right_lost_center));
    memset(top_lost_center, 0, sizeof(top_lost_center));
    memset(bottom_lost_center, 0, sizeof(bottom_lost_center));

    camera_count_lost_blocks_vertical(&top_lost_count, &top_lost_num, top_lost_center,
                                      CAMERA_TOP_START_ROW, CAMERA_TOP_END_ROW, 0);
    camera_count_lost_blocks_vertical(&bottom_lost_count, &bottom_lost_num, bottom_lost_center,
                                      CAMERA_BOTTOM_START_ROW, MT9V03X_1_H, 0);
    camera_count_lost_blocks_vertical(&left_lost_count, &left_lost_num, left_lost_center,
                                      CAMERA_TOP_START_ROW, MT9V03X_1_H, 1);
    camera_count_lost_blocks_vertical(&right_lost_count, &right_lost_num, right_lost_center,
                                      CAMERA_TOP_START_ROW, MT9V03X_1_H, 2);
}

static void camera_enter_phase(uint8 new_phase)
{
    vision_phase = new_phase;
}

static uint8 camera_phase_is_corner(uint8 phase)
{
    return (phase == VISION_PHASE_CORNER_L_ENTER ||
            phase == VISION_PHASE_CORNER_L_TURN ||
            phase == VISION_PHASE_CORNER_L_EXIT ||
            phase == VISION_PHASE_CORNER_R_ENTER ||
            phase == VISION_PHASE_CORNER_R_TURN ||
            phase == VISION_PHASE_CORNER_R_EXIT);
}

static uint8 camera_corner_phase_from_dir(uint8 dir, uint8 stage)
{
    if(dir == 1)
    {
        if(stage == 0) return VISION_PHASE_CORNER_L_ENTER;
        if(stage == 1) return VISION_PHASE_CORNER_L_TURN;
        return VISION_PHASE_CORNER_L_EXIT;
    }

    if(stage == 0) return VISION_PHASE_CORNER_R_ENTER;
    if(stage == 1) return VISION_PHASE_CORNER_R_TURN;
    return VISION_PHASE_CORNER_R_EXIT;
}

static void camera_update_public_state_from_phase(void)
{
    if(vision_phase == VISION_PHASE_COMPONENT_ENTER ||
       vision_phase == VISION_PHASE_COMPONENT_LOCK ||
       vision_phase == VISION_PHASE_COMPONENT_EXIT)
    {
        vision_state = VISION_COMPONENT;
        corner_type = 0;
        return;
    }

    if(vision_phase == VISION_PHASE_CORNER_L_ENTER ||
       vision_phase == VISION_PHASE_CORNER_L_TURN ||
       vision_phase == VISION_PHASE_CORNER_L_EXIT)
    {
        vision_state = VISION_CORNER_LEFT;
        corner_type = 1;
        corner_direction = 1;
        return;
    }

    if(vision_phase == VISION_PHASE_CORNER_R_ENTER ||
       vision_phase == VISION_PHASE_CORNER_R_TURN ||
       vision_phase == VISION_PHASE_CORNER_R_EXIT)
    {
        vision_state = VISION_CORNER_RIGHT;
        corner_type = 2;
        corner_direction = 2;
        return;
    }

    if(vision_phase == VISION_PHASE_LOST)
    {
        vision_state = VISION_LOST;
        corner_type = 0;
        corner_direction = 0;
        return;
    }

    vision_state = VISION_NORMAL;
    corner_type = 0;
    corner_direction = 0;
}

static uint8 camera_last_stored_block_index(uint8 block_num);

static uint8 camera_side_lost_center_valid(uint8 block_num, uint8 centers[])
{
    uint8 center;

    if(block_num == 0)
    {
        return 0;
    }

    center = centers[camera_last_stored_block_index(block_num)];
    return (center > CAMERA_TOP_START_ROW &&
            center < (CAMERA_BOTTOM_START_ROW - CAMERA_LOST_BLOCK_MIN_LEN));
}

static uint8 camera_detect_raw_corner(uint8 bottom_valid)
{
    uint8 left_valid;
    uint8 right_valid;

    if(bottom_valid < CAMERA_CORNER_BOTTOM_VALID_MIN)
    {
        return 0;
    }

    if(component_row_count >= CAMERA_CORNER_COMPONENT_BLOCK)
    {
        return 0;
    }

    if(top_lost_count < CAMERA_CORNER_TOP_LOST_MIN || top_lost_num == 0)
    {
        return 0;
    }

    left_valid = (left_lost_num >= CAMERA_CORNER_SIDE_BLOCK_MIN &&
                  left_lost_count >= CAMERA_CORNER_SIDE_LOST_MIN &&
                  left_lost_count >= right_lost_count + CAMERA_CORNER_SIDE_DOMINANCE &&
                  right_lost_count <= CAMERA_CORNER_OPPOSITE_MAX &&
                  camera_side_lost_center_valid(left_lost_num, left_lost_center));

    right_valid = (right_lost_num >= CAMERA_CORNER_SIDE_BLOCK_MIN &&
                   right_lost_count >= CAMERA_CORNER_SIDE_LOST_MIN &&
                   right_lost_count >= left_lost_count + CAMERA_CORNER_SIDE_DOMINANCE &&
                   left_lost_count <= CAMERA_CORNER_OPPOSITE_MAX &&
                   camera_side_lost_center_valid(right_lost_num, right_lost_center));

    if(left_valid && !right_valid)
    {
        return 1;
    }
    if(right_valid && !left_valid)
    {
        return 2;
    }

    return 0;
}

static uint8 camera_detect_early_corner(uint8 bottom_valid)
{
    if(bottom_valid < CAMERA_CORNER_BOTTOM_VALID_MIN)
    {
        return 0;
    }
    if(component_row_count >= CAMERA_CORNER_COMPONENT_BLOCK)
    {
        return 0;
    }

    // 草莽思路：一侧贴边但顶部还没丢时提前识别直角，不等线从顶部完全消失。
    if(left_lost_num > 0 &&
       left_lost_count >= CAMERA_CORNER_EARLY_SIDE_LOST_MIN &&
       right_lost_num == 0 &&
       top_lost_count < CAMERA_CORNER_EARLY_TOP_LOST_MAX &&
       camera_side_lost_center_valid(left_lost_num, left_lost_center))
    {
        return 1;
    }

    if(right_lost_num > 0 &&
       right_lost_count >= CAMERA_CORNER_EARLY_SIDE_LOST_MIN &&
       left_lost_num == 0 &&
       top_lost_count < CAMERA_CORNER_EARLY_TOP_LOST_MAX &&
       camera_side_lost_center_valid(right_lost_num, right_lost_center))
    {
        return 2;
    }

    return 0;
}

static void camera_update_vision_state(void)
{
    static uint8 phase_ticks = 0;
    static uint8 exit_confirm = 0;
    static uint8 lost_confirm = 0;
    uint8 bottom_valid = count_valid_rows(CAMERA_BOTTOM_START_ROW, MT9V03X_1_H);
    uint8 normal_valid = count_valid_rows(MT9V03X_1_H / 3, MT9V03X_1_H * 5 / 6);
    uint8 raw_corner = 0;
    uint8 component_detected;
    uint8 normal_ready;
    uint8 lost_detected;

    component_detected = (component_row_count >= CAMERA_COMPONENT_ROWS_MIN);
    raw_corner = camera_detect_raw_corner(bottom_valid);
    if(raw_corner == 0)
    {
        raw_corner = camera_detect_early_corner(bottom_valid);
    }
    normal_ready = (normal_valid >= CAMERA_CORNER_EXIT_VALID_MIN &&
                    left_lost_num == 0 &&
                    right_lost_num == 0 &&
                    top_lost_count < (CAMERA_CORNER_TOP_LOST_MIN / 2));
    lost_detected = (normal_valid < 4 && !component_detected && raw_corner == 0);

    if(raw_corner != 0 && !camera_phase_is_corner(vision_phase))
    {
        corner_direction = raw_corner;
        camera_enter_phase(camera_corner_phase_from_dir(raw_corner, 0));
        phase_ticks = 0;
        exit_confirm = 0;
        lost_confirm = 0;
        camera_update_public_state_from_phase();
        return;
    }

    if(lost_detected)
    {
        if(lost_confirm < 255) lost_confirm++;
    }
    else
    {
        lost_confirm = 0;
    }

    switch(vision_phase)
    {
        case VISION_PHASE_NORMAL:
            if(component_detected)
            {
                camera_enter_phase(VISION_PHASE_COMPONENT_ENTER);
                phase_ticks = 0;
                exit_confirm = 0;
            }
            else if(lost_confirm >= CAMERA_LOST_CONFIRM_FRAMES)
            {
                camera_enter_phase(VISION_PHASE_LOST);
                phase_ticks = 0;
                exit_confirm = 0;
            }
            break;

        case VISION_PHASE_COMPONENT_ENTER:
            if(raw_corner != 0)
            {
                corner_direction = raw_corner;
                camera_enter_phase(camera_corner_phase_from_dir(raw_corner, 0));
                phase_ticks = 0;
                exit_confirm = 0;
                break;
            }
            if(component_detected || phase_ticks >= CAMERA_COMPONENT_ENTER_FRAMES)
            {
                camera_enter_phase(VISION_PHASE_COMPONENT_LOCK);
                phase_ticks = 0;
                exit_confirm = 0;
            }
            break;

        case VISION_PHASE_COMPONENT_LOCK:
            if(raw_corner != 0)
            {
                corner_direction = raw_corner;
                camera_enter_phase(camera_corner_phase_from_dir(raw_corner, 0));
                phase_ticks = 0;
                exit_confirm = 0;
                break;
            }
            if(phase_ticks >= CAMERA_COMPONENT_LOCK_MIN && !component_detected && normal_ready)
            {
                if(exit_confirm < 255) exit_confirm++;
                if(exit_confirm >= CAMERA_COMPONENT_EXIT_FRAMES)
                {
                    camera_enter_phase(VISION_PHASE_COMPONENT_EXIT);
                    phase_ticks = 0;
                    exit_confirm = 0;
                }
            }
            else
            {
                exit_confirm = 0;
            }
            break;

        case VISION_PHASE_COMPONENT_EXIT:
            if(component_detected)
            {
                camera_enter_phase(VISION_PHASE_COMPONENT_LOCK);
                phase_ticks = 0;
                exit_confirm = 0;
            }
            else if(raw_corner != 0)
            {
                corner_direction = raw_corner;
                camera_enter_phase(camera_corner_phase_from_dir(raw_corner, 0));
                phase_ticks = 0;
                exit_confirm = 0;
            }
            else if(phase_ticks >= CAMERA_COMPONENT_EXIT_FRAMES)
            {
                camera_enter_phase(VISION_PHASE_NORMAL);
                phase_ticks = 0;
                exit_confirm = 0;
            }
            break;

        case VISION_PHASE_CORNER_L_ENTER:
        case VISION_PHASE_CORNER_R_ENTER:
            if(phase_ticks >= CAMERA_CORNER_ENTER_FRAMES)
            {
                camera_enter_phase(camera_corner_phase_from_dir(corner_direction, 1));
                phase_ticks = 0;
                exit_confirm = 0;
            }
            break;

        case VISION_PHASE_CORNER_L_TURN:
        case VISION_PHASE_CORNER_R_TURN:
            if(phase_ticks >= CAMERA_CORNER_TURN_MIN_FRAMES && normal_ready)
            {
                if(exit_confirm < 255) exit_confirm++;
                if(exit_confirm >= CAMERA_CORNER_EXIT_FRAMES)
                {
                    camera_enter_phase(camera_corner_phase_from_dir(corner_direction, 2));
                    phase_ticks = 0;
                    exit_confirm = 0;
                }
            }
            else
            {
                exit_confirm = 0;
            }
            break;

        case VISION_PHASE_CORNER_L_EXIT:
        case VISION_PHASE_CORNER_R_EXIT:
            if(raw_corner != 0 && raw_corner == corner_direction)
            {
                camera_enter_phase(camera_corner_phase_from_dir(corner_direction, 1));
                phase_ticks = 0;
                exit_confirm = 0;
            }
            else if(phase_ticks >= CAMERA_CORNER_EXIT_FRAMES)
            {
                camera_enter_phase(VISION_PHASE_NORMAL);
                phase_ticks = 0;
                exit_confirm = 0;
            }
            break;

        case VISION_PHASE_LOST:
            if(raw_corner != 0)
            {
                corner_direction = raw_corner;
                camera_enter_phase(camera_corner_phase_from_dir(raw_corner, 0));
                phase_ticks = 0;
                exit_confirm = 0;
            }
            else if(component_detected)
            {
                camera_enter_phase(VISION_PHASE_COMPONENT_ENTER);
                phase_ticks = 0;
                exit_confirm = 0;
            }
            else if(normal_ready)
            {
                camera_enter_phase(VISION_PHASE_NORMAL);
                phase_ticks = 0;
                exit_confirm = 0;
            }
            break;

        default:
            camera_enter_phase(VISION_PHASE_NORMAL);
            phase_ticks = 0;
            exit_confirm = 0;
            break;
    }

    if(phase_ticks < 255) phase_ticks++;
    camera_update_public_state_from_phase();
}
static int16 find_bottom_stable_mid(void)
{
    for(int y = MT9V03X_1_H - 1; y >= CAMERA_BOTTOM_START_ROW; y--)
    {
        if(line_found[y])
        {
            return process_line_mid[y];
        }
    }
    return LOST_LINE_REPLACE_VAL;
}

static int camera_find_bottom_stable_y(void)
{
    for(int y = MT9V03X_1_H - 1; y >= CAMERA_BOTTOM_START_ROW; y--)
    {
        if(line_found[y])
        {
            return y;
        }
    }
    return MT9V03X_1_H - 1;
}

static uint8 camera_last_stored_block_index(uint8 block_num)
{
    if(block_num == 0) return 0;
    if(block_num > CAMERA_LOST_BLOCK_MAX) return CAMERA_LOST_BLOCK_MAX - 1;
    return block_num - 1;
}

static int camera_select_corner_end_y(uint8 direction)
{
    int end_y = MT9V03X_1_H / 3;

    if(direction == 1 && left_lost_num > 0)
    {
        end_y = left_lost_center[camera_last_stored_block_index(left_lost_num)];
    }
    else if(direction == 2 && right_lost_num > 0)
    {
        end_y = right_lost_center[camera_last_stored_block_index(right_lost_num)];
    }
    else if(top_lost_num > 0)
    {
        end_y = top_lost_center[0];
    }

    if(end_y < MT9V03X_1_H / 4) end_y = MT9V03X_1_H / 4;
    if(end_y > MT9V03X_1_H * 2 / 3) end_y = MT9V03X_1_H * 2 / 3;
    return end_y;
}

static int16 camera_curve_patch_x(int16 start_x, int16 end_x, int start_y, int end_y, int y)
{
    int total = start_y - end_y;
    int progress = start_y - y;
    int32 x;

    if(total <= 0)
    {
        return start_x;
    }

    x = (int32)start_x + ((int32)(end_x - start_x) * progress) / total;
    return clamp_image_x(x);
}

static void camera_extend_component_line(void)
{
    int16 carry_mid = find_bottom_stable_mid();

    for(int y = MT9V03X_1_H - 1; y >= 0; y--)
    {
        if(line_found[y] && !component_row[y])
        {
            carry_mid = process_line_mid[y];
        }
        else if(component_row[y])
        {
            process_line_mid[y] = carry_mid;
        }
    }
}

static void camera_apply_corner_patch(void)
{
    int start_y;
    int end_y;
    int rows;
    int black_count = 0;
    int bottom_valid;
    int score = 0;
    int16 start_x;
    int16 end_x;

    corner_patch_quality = 0;

    if(vision_state != VISION_CORNER_LEFT && vision_state != VISION_CORNER_RIGHT)
    {
        return;
    }

    start_y = camera_find_bottom_stable_y();
    end_y = camera_select_corner_end_y(corner_direction);
    start_x = find_bottom_stable_mid();
    end_x = (vision_state == VISION_CORNER_LEFT) ?
            (CAMERA_EDGE_LOST_MARGIN + 2) :
            (MT9V03X_1_W - 1 - CAMERA_EDGE_LOST_MARGIN - 2);

    if(end_y > CAMERA_CORNER_PATCH_END_ROW)
    {
        end_y = CAMERA_CORNER_PATCH_END_ROW;
    }

    if(end_y >= start_y)
    {
        return;
    }

    rows = start_y - end_y + 1;
    if(rows < CAMERA_CORNER_SIDE_LOST_MIN)
    {
        return;
    }

    if((vision_state == VISION_CORNER_LEFT && end_x >= start_x) ||
       (vision_state == VISION_CORNER_RIGHT && end_x <= start_x))
    {
        return;
    }

    for(int y = start_y; y >= end_y; y--)
    {
        int16 patched_mid = camera_curve_patch_x(start_x, end_x, start_y, end_y, y);
        if(process_image[y][patched_mid] == 0)
        {
            black_count++;
        }
    }

    if(black_count > rows * CAMERA_PATCH_BLACK_MAX_PERCENT / 100)
    {
        return;
    }

    score = 30;
    bottom_valid = count_valid_rows(CAMERA_BOTTOM_START_ROW, MT9V03X_1_H);
    score += (bottom_valid > 20) ? 20 : bottom_valid;
    if(top_lost_num > 0) score += 15;
    if((vision_state == VISION_CORNER_LEFT && left_lost_num > 0) ||
       (vision_state == VISION_CORNER_RIGHT && right_lost_num > 0))
    {
        score += 30;
    }
    if(black_count < rows * 2 / 3)
    {
        score += 10;
    }
    if(score > 100) score = 100;
    corner_patch_quality = (uint8)score;

    if(corner_patch_quality < CAMERA_PATCH_QUALITY_MIN)
    {
        return;
    }

    for(int y = start_y; y >= end_y; y--)
    {
        int16 patched_mid = camera_curve_patch_x(start_x, end_x, start_y, end_y, y);
        process_line_mid[y] = patched_mid;
        line_mid[y] = patched_mid;
        line_found[y] = 1;
        component_row[y] = 0;
    }
}

static void camera_update_track_offset(void)
{
    static int16 last_offset = 0;
    int32 sum = 0;
    int32 weight_sum = 0;
    int start;
    int end;
    int16 raw_offset;
    int16 diff;
    int16 step_limit = CAMERA_OFFSET_STEP_MAX;

    if(vision_state == VISION_CORNER_LEFT || vision_state == VISION_CORNER_RIGHT)
    {
        start = MT9V03X_1_H / 3;
        end = MT9V03X_1_H * 2 / 3;
        step_limit = CAMERA_CORNER_OFFSET_STEP_MAX;
    }
    else
    {
        start = MT9V03X_1_H / 3;
        end = MT9V03X_1_H * 5 / 6;
        if(vision_state == VISION_COMPONENT)
        {
            step_limit = CAMERA_COMPONENT_OFFSET_STEP;
        }
    }

    for(int y = start; y < end; y++)
    {
        if(!line_found[y] || component_row[y])
        {
            continue;
        }

        sum += (process_line_mid[y] - (MT9V03X_1_W / 2)) * y;
        weight_sum += y;
    }

    raw_offset = (weight_sum > 0) ? (int16)(sum / weight_sum) : last_offset;
    diff = raw_offset - last_offset;

    if(diff > step_limit) diff = step_limit;
    if(diff < -step_limit) diff = -step_limit;

    track_offset = last_offset + diff;
    last_offset = track_offset;
}

void image_process_task(void)
{
    if(mt9v03x_finish_flag_1 == 1)
    {
        camera_copy_stable_frame();
        mt9v03x_finish_flag_1 = 0;

        camera_reset_frame_features();
        camera_binarize_frame();
        camera_extract_row_segments();
        camera_split_wide_segments();
        camera_trace_connected_track();
        camera_update_edge_migration();
        camera_update_lost_features();
        camera_update_vision_state();
        camera_extend_component_line();
        camera_apply_corner_patch();
        camera_update_track_offset();

        for(int y = 0; y < MT9V03X_1_H; y++)
        {
            line_mid[y] = process_line_mid[y];
        }

        image_swap_buffer();
        image_ready = 1;
    }
}

void image_display_task(void)
{
#if ENABLE_DISPLAY
    static uint8 refresh_cnt = 0;
    if(!image_ready) return;

    if(++refresh_cnt < DISPLAY_REFRESH_SKIP) return;
    refresh_cnt = 0;

    ips200_show_gray_image(0, 0, display_image[0], MT9V03X_1_W, MT9V03X_1_H,
                           MT9V03X_1_W / DISPLAY_IMAGE_DIV,
                           MT9V03X_1_H / DISPLAY_IMAGE_DIV,
                           0);

    for(int y = 0; y < MT9V03X_1_H; y += DISPLAY_IMAGE_DIV)
    {
        if(display_line_mid[y] >= 0 && display_line_mid[y] < MT9V03X_1_W)
        {
            ips200_draw_point((uint16)(display_line_mid[y] / DISPLAY_IMAGE_DIV),
                              (uint16)(y / DISPLAY_IMAGE_DIV),
                              RGB565_RED);
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

uint8 Camera_GetVisionState(void)
{
    return vision_state;
}

uint8 Camera_GetVisionPhase(void)
{
    return vision_phase;
}

uint8 Camera_GetCornerDirection(void)
{
    return corner_direction;
}

uint8 Camera_GetLeftLostCount(void)
{
    return left_lost_count;
}

uint8 Camera_GetRightLostCount(void)
{
    return right_lost_count;
}

uint8 Camera_GetTopLostCount(void)
{
    return top_lost_count;
}

uint8 Camera_GetBottomLostCount(void)
{
    return bottom_lost_count;
}

uint8 Camera_GetLeftLostBlockCount(void)
{
    return left_lost_num;
}

uint8 Camera_GetRightLostBlockCount(void)
{
    return right_lost_num;
}

uint8 Camera_GetTopLostBlockCount(void)
{
    return top_lost_num;
}

uint8 Camera_GetBottomLostBlockCount(void)
{
    return bottom_lost_num;
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
