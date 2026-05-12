#include "process_img.h"

#include <stdlib.h>

#define PROCESS_MM_PER_COUNT 0.003897f

uint8 imgGray[YM][XM];
uint8 img2wifi[YM][XM];
uint8 process_img_last_threshold = 0;
float Length = 0.0f;

static uint8 process_img_compute_otsu(const uint8 image[YM][XM])
{
    uint32 histogram[256];
    uint32 total_pixels;
    uint32 sum_total;
    uint32 sum_background;
    uint32 weight_background;
    uint32 weight_foreground;
    uint32 level;
    uint32 x;
    uint32 y;
    float best_variance;
    uint8 best_threshold;

    for(level = 0; level < 256; level++)
    {
        histogram[level] = 0;
    }

    sum_total = 0;
    total_pixels = XM * YM;
    for(y = 0; y < YM; y++)
    {
        for(x = 0; x < XM; x++)
        {
            uint8 gray = image[y][x];
            histogram[gray]++;
            sum_total += gray;
        }
    }

    best_variance = -1.0f;
    best_threshold = 0;
    sum_background = 0;
    weight_background = 0;

    for(level = 0; level < 256; level++)
    {
        float mean_background;
        float mean_foreground;
        float delta;
        float variance;

        weight_background += histogram[level];
        if(weight_background == 0)
        {
            continue;
        }

        weight_foreground = total_pixels - weight_background;
        if(weight_foreground == 0)
        {
            break;
        }

        sum_background += level * histogram[level];
        mean_background = (float)sum_background / (float)weight_background;
        mean_foreground = (float)(sum_total - sum_background) / (float)weight_foreground;
        delta = mean_background - mean_foreground;
        variance = (float)weight_background * (float)weight_foreground * delta * delta;

        if(variance > best_variance)
        {
            best_variance = variance;
            best_threshold = (uint8)level;
        }
    }

    return best_threshold;
}

static void process_img_copy_and_compress_stable_frame(void)
{
    uint32 interrupt_state;
    uint32 x;
    uint32 y;

    interrupt_state = interrupt_global_disable();
    if(mt9v03x_finish_flag_1)
    {
        for(y = 0; y < YM; y++)
        {
            uint32 src_y = y * 2U;
            for(x = 0; x < XM; x++)
            {
                uint32 src_x = x * 2U;
                imgGray[y][x] = mt9v03x_image_2[src_y][src_x];
            }
        }
        mt9v03x_finish_flag_1 = 0;
    }
    interrupt_global_enable(interrupt_state);
}

static void process_img_binarize_otsu(const uint8 input[YM][XM], uint8 output[YM][XM], uint8 threshold)
{
    uint32 x;
    uint32 y;

    for(y = 0; y < YM; y++)
    {
        for(x = 0; x < XM; x++)
        {
            if(0U == y || (YM - 1U) == y || 0U == x || (XM - 1U) == x)
            {
                output[y][x] = 0;
            }
            else
            {
                output[y][x] = (input[y][x] >= threshold) ? 255 : 0;
            }
        }
    }
}

void ProcessImg_Init(void)
{
    uint32 x;
    uint32 y;

    process_img_last_threshold = 0;
    Length = 0.0f;
    for(y = 0; y < YM; y++)
    {
        for(x = 0; x < XM; x++)
        {
            imgGray[y][x] = 0;
            img2wifi[y][x] = 0;
        }
    }
}

uint8 ProcessImg_ProcessFrame(void)
{
    if(!mt9v03x_finish_flag_1)
    {
        return 0;
    }

    process_img_copy_and_compress_stable_frame();
    process_img_last_threshold = process_img_compute_otsu(imgGray);
    process_img_binarize_otsu(imgGray, img2wifi, process_img_last_threshold);

    return 1;
}

void ProcessImg_UpdateMotion10ms(int16 speed_l, int16 speed_r)
{
    uint32 mean_abs_count;

    mean_abs_count = ((uint32)abs(speed_l) + (uint32)abs(speed_r)) / 2U;
    Length += (float)mean_abs_count * PROCESS_MM_PER_COUNT;
}
