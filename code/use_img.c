#include "use_img.h"
#include "process_img.h"
#include "wifi_img.h"
#include "zf_device_ips200.h"
#include "zf_device_mt9v03x_double.h"
#include <string.h>

#define DISPLAY_IMAGE_DIV    1
#define DISPLAY_REFRESH_SKIP 10
#define CONTROL_LOOKAHEAD_ROW ((Deal_Bottom + Deal_Top) / 2)

typedef struct
{
    uint8 valid;
    uint8 centerpoint_target;
    uint8 phase;
    uint32 l_data_statics;
    uint32 r_data_statics;
    uint8 mid_line[YM];
} VisualControlSnapshot_t;

static volatile VisualControlSnapshot_t visual_snapshot = {0};

static void fill_mid_line_buffer(uint8 *buffer, uint8 value)
{
    for(uint16 y = 0; y < YM; y++)
    {
        buffer[y] = value;
    }
}

static uint8 compute_centerpoint_target(void)
{
    int32 sum = 0;
    int32 count = 0;

    for(int y = CONTROL_LOOKAHEAD_ROW - 6; y <= CONTROL_LOOKAHEAD_ROW + 6; y++)
    {
        if(y < Deal_Bottom || y > Deal_Top)
        {
            continue;
        }

        if(imgOSTU[y][mid_line[y]] == Control_line || imgOSTU[y][mid_line[y]] == Judge_line)
        {
            sum += mid_line[y];
            count++;
        }
    }

    if(count == 0)
    {
        return mid_line[CONTROL_LOOKAHEAD_ROW];
    }

    return (uint8)(sum / count);
}

static void publish_visual_snapshot(void)
{
    VisualControlSnapshot_t next_snapshot;

    next_snapshot.valid = 1;
    next_snapshot.centerpoint_target = compute_centerpoint_target();
    next_snapshot.phase = (uint8)IF;
    next_snapshot.l_data_statics = l_data_statics;
    next_snapshot.r_data_statics = r_data_statics;
    for(uint16 y = 0; y < YM; y++)
    {
        next_snapshot.mid_line[y] = mid_line[y];
    }

    uint32 interrupt_state = interrupt_global_disable();
    visual_snapshot.valid = next_snapshot.valid;
    visual_snapshot.centerpoint_target = next_snapshot.centerpoint_target;
    visual_snapshot.phase = next_snapshot.phase;
    visual_snapshot.l_data_statics = next_snapshot.l_data_statics;
    visual_snapshot.r_data_statics = next_snapshot.r_data_statics;
    for(uint16 y = 0; y < YM; y++)
    {
        visual_snapshot.mid_line[y] = next_snapshot.mid_line[y];
    }

    interrupt_global_enable(interrupt_state);
}

static void invalidate_visual_snapshot(void)
{
    VisualControlSnapshot_t next_snapshot;

    next_snapshot.valid = 0;
    next_snapshot.centerpoint_target = (uint8)(XM / 2);
    next_snapshot.phase = straightlineS;
    next_snapshot.l_data_statics = 0;
    next_snapshot.r_data_statics = 0;
    fill_mid_line_buffer(next_snapshot.mid_line, (uint8)(XM / 2));

    uint32 interrupt_state = interrupt_global_disable();
    visual_snapshot.valid = next_snapshot.valid;
    visual_snapshot.centerpoint_target = next_snapshot.centerpoint_target;
    visual_snapshot.phase = next_snapshot.phase;
    visual_snapshot.l_data_statics = next_snapshot.l_data_statics;
    visual_snapshot.r_data_statics = next_snapshot.r_data_statics;
    for(uint16 y = 0; y < YM; y++)
    {
        visual_snapshot.mid_line[y] = next_snapshot.mid_line[y];
    }

    interrupt_global_enable(interrupt_state);
}

static void process_regular_track(void)
{
    Get_start_center();

    if(b_lost_num > 0 && t_lost_num > 0)
    {
        Stop_line = Get_top_straightline();
    }
    else if(b_lost_num > 0 && l_lost_num > 0 && t_lost_num == 0 && r_lost_num == 0)
    {
        Stop_line = Left_curve_line();
    }
    else if(b_lost_num > 0 && r_lost_num > 0 && t_lost_num == 0 && l_lost_num == 0)
    {
        Stop_line = Right_curve_line();
    }
    else
    {
        Stop_line = 0;
        IF = curve;
    }
}

static void show_imgOSTU(void)
{
    static uint8 refresh_cnt = 0;

    if(++refresh_cnt < DISPLAY_REFRESH_SKIP)
    {
        return;
    }
    refresh_cnt = 0;

    ips200_show_gray_image(0, 0, imgGray[0], XM, YM,
                           XM / DISPLAY_IMAGE_DIV,
                           YM / DISPLAY_IMAGE_DIV,
                           0);

    for(int y = 0; y < YM; y += DISPLAY_IMAGE_DIV)
    {
        for(int x = 0; x < XM; x += DISPLAY_IMAGE_DIV)
        {
            uint8 pixel = imgOSTU[y][x];
            uint16 color = 0;

            if(pixel == Control_line)
            {
                color = RGB565_RED;
            }
            else if(pixel == Judge_line)
            {
                color = RGB565_YELLOW;
            }
            else if(pixel == Left_line)
            {
                color = RGB565_GREEN;
            }
            else if(pixel == Right_line)
            {
                color = RGB565_BLUE;
            }
            else if(pixel == Lost_line)
            {
                color = RGB565_PURPLE;
            }

            if(color != 0)
            {
                ips200_draw_point((uint16)(x / DISPLAY_IMAGE_DIV),
                                  (uint16)(y / DISPLAY_IMAGE_DIV),
                                  color);
            }
        }
    }
}

void use_img_init(void)
{
    invalidate_visual_snapshot();
    mt9v03x_double_init(mt9v03x_1);
}

void use_img(void)
{
    if(mt9v03x_finish_flag_1 == 1)
    {
        deal_runing = 1;
        dealimg_finish_flag = 0;
        mt9v03x_finish_flag_1 = 0;

        standard();
        nowThreshold = getOSTUThreshold();
        Get_imgOSTU();
        Dashedline_Makeup();

        IF_L = IF;
        Stop_line = 0;
        memset(mid_line, XM / 2, sizeof(mid_line));

        if(disappear_flag == 0 && get_start_point())
        {
            search_l_r(start_point_l, start_center_y, start_point_r, start_center_y);
            Get_lost_tip(2);

            Annulus_flag = Go_Annulus();

            if(!Annulus_flag)
            {
                Stop_line = Deal_crossroads();
            }

            if(Stop_line == 0 && !Annulus_flag)
            {
                process_regular_track();
            }

            stopline_flag = (uint8)(stopline_flag + Get_stopline());
        }

        disappear_flag = Deal_disappear();
        if(mid_line_valid)
        {
            publish_visual_snapshot();
        }
        else
        {
            invalidate_visual_snapshot();
        }
        wifi_img_send();
        deal_runing = 0;
        dealimg_finish_flag = 1;
    }

    show_imgOSTU();
}

uint8 use_img_is_ready(void)
{
    return visual_snapshot.valid;
}

uint8 use_img_snapshot_centerpoint_target(void)
{
    return visual_snapshot.centerpoint_target;
}

uint8 use_img_snapshot_mid_line(uint8 y)
{
    if(y >= YM)
    {
        return (uint8)(XM / 2);
    }
    return visual_snapshot.mid_line[y];
}

uint32 use_img_snapshot_l_data_statics(void)
{
    return visual_snapshot.l_data_statics;
}

uint32 use_img_snapshot_r_data_statics(void)
{
    return visual_snapshot.r_data_statics;
}

uint8 use_img_snapshot_phase(void)
{
    return visual_snapshot.phase;
}
