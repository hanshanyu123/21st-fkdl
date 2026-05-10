#include "use_img.h"
#include "process_img.h"
#include "zf_device_ips200.h"
#include "zf_device_mt9v03x_double.h"
#include <string.h>

#define DISPLAY_IMAGE_DIV    2
#define DISPLAY_REFRESH_SKIP 10

static uint8 frame_ready = 0;

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

    if(!frame_ready)
    {
        return;
    }
    if(++refresh_cnt < DISPLAY_REFRESH_SKIP)
    {
        return;
    }
    refresh_cnt = 0;

    ips200_show_gray_image(0, 0, imgOSTU[0], XM, YM,
                           XM / DISPLAY_IMAGE_DIV,
                           YM / DISPLAY_IMAGE_DIV,
                           0);

    for(int y = 0; y < YM; y += DISPLAY_IMAGE_DIV)
    {
        ips200_draw_point((uint16)(mid_line[y] / DISPLAY_IMAGE_DIV),
                          (uint16)(y / DISPLAY_IMAGE_DIV),
                          RGB565_RED);
    }
}

void use_img_init(void)
{
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
        deal_runing = 0;
        dealimg_finish_flag = 1;
        frame_ready = 1;
    }

    show_imgOSTU();
}

uint8 use_img_is_ready(void)
{
    return frame_ready;
}
