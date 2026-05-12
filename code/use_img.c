#include "use_img.h"

#include "process_img.h"
#include "wifi_img.h"
#include "zf_device_ips200.h"

#define USE_IMG_DISPLAY_X 0
#define USE_IMG_DISPLAY_Y 0
#define USE_IMG_DISPLAY_W (XM * 2)
#define USE_IMG_DISPLAY_H (YM * 2)

static uint8 use_img_ready = 0;
static uint8 use_img_snapshot_phase_value = straight;
static uint8 use_img_snapshot_l_statics = 0;
static uint8 use_img_snapshot_r_statics = 0;
static uint8 use_img_snapshot_center_target = (XM / 2);

static void use_img_show_binary_result(void)
{
    ips200_show_gray_image(USE_IMG_DISPLAY_X,
                           USE_IMG_DISPLAY_Y,
                           img2wifi[0],
                           XM,
                           YM,
                           USE_IMG_DISPLAY_W,
                           USE_IMG_DISPLAY_H,
                           128);
    ips200_show_string(0, USE_IMG_DISPLAY_H + 4, "OTSU BIN");
    ips200_show_uint(72, USE_IMG_DISPLAY_H + 4, process_img_last_threshold, 3);
}

void use_img_init(void)
{
    while(mt9v03x_double_init(mt9v03x_1))
    {
        ips200_clear();
        ips200_show_string(0, 0, "CAM INIT FAIL");
        ips200_show_string(0, 20, "RETRY...");
        system_delay_ms(200);
    }

    ProcessImg_Init();
    use_img_ready = 0;
    use_img_snapshot_phase_value = straight;
    use_img_snapshot_l_statics = 0;
    use_img_snapshot_r_statics = 0;
    use_img_snapshot_center_target = (XM / 2);
    ips200_clear();
    ips200_show_string(0, 0, "WAIT FRAME");
}

void use_img(void)
{
    if(ProcessImg_ProcessFrame())
    {
        use_img_show_binary_result();
        wifi_img_send();
    }
}

uint8 use_img_is_ready(void)
{
    return use_img_ready;
}

uint8 use_img_snapshot_phase(void)
{
    return use_img_snapshot_phase_value;
}

uint8 use_img_snapshot_l_data_statics(void)
{
    return use_img_snapshot_l_statics;
}

uint8 use_img_snapshot_r_data_statics(void)
{
    return use_img_snapshot_r_statics;
}

uint8 use_img_snapshot_centerpoint_target(void)
{
    return use_img_snapshot_center_target;
}
