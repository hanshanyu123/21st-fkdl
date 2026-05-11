#include "wifi_img.h"

#include "process_img.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_wifi_spi.h"

#define WIFI_IMG_RETRY_MS 100U

typedef enum
{
    WIFI_IMG_STAGE_WIFI_INIT = 0,
    WIFI_IMG_STAGE_SOCKET_CONNECT,
    WIFI_IMG_STAGE_ASSISTANT_INIT,
    WIFI_IMG_STAGE_READY,
} wifi_img_stage_enum;

static wifi_img_stage_enum wifi_img_stage = WIFI_IMG_STAGE_WIFI_INIT;
static uint32 wifi_img_last_try_ms = 0;
static uint8 wifi_img_info_printed = 0;

static uint8 wifi_img_retry_elapsed(uint32 now_ms)
{
    return ((uint32)(now_ms - wifi_img_last_try_ms) >= WIFI_IMG_RETRY_MS);
}

static void wifi_img_print_module_info(void)
{
    if(wifi_img_info_printed)
    {
        return;
    }

    printf("\r\n module version:%s", wifi_spi_version);
    printf("\r\n module mac    :%s", wifi_spi_mac_addr);
    printf("\r\n module ip     :%s", wifi_spi_ip_addr_port);
    wifi_img_info_printed = 1;
}

void wifi_img_init(void)
{
    wifi_img_stage = WIFI_IMG_STAGE_WIFI_INIT;
    wifi_img_last_try_ms = 0;
    wifi_img_info_printed = 0;
}

void wifi_img_task(void)
{
    uint32 now_ms = system_getval_ms();

    if(WIFI_IMG_STAGE_READY == wifi_img_stage)
    {
        return;
    }

    if(!wifi_img_retry_elapsed(now_ms))
    {
        return;
    }
    wifi_img_last_try_ms = now_ms;

    if(WIFI_IMG_STAGE_WIFI_INIT == wifi_img_stage)
    {
        if(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
        {
            printf("\r\n connect wifi failed. \r\n");
            return;
        }

        wifi_img_print_module_info();
        wifi_img_stage = WIFI_IMG_STAGE_SOCKET_CONNECT;
    }

    if(WIFI_IMG_STAGE_SOCKET_CONNECT == wifi_img_stage)
    {
        if(1 != WIFI_SPI_AUTO_CONNECT)
        {
            if(wifi_spi_socket_connect("TCP",
                                       WIFI_SPI_TARGET_IP,
                                       WIFI_SPI_TARGET_PORT,
                                       WIFI_SPI_LOCAL_PORT))
            {
                printf("\r\n Connect TCP Servers error, try again.");
                return;
            }
        }

        wifi_img_stage = WIFI_IMG_STAGE_ASSISTANT_INIT;
    }

    if(WIFI_IMG_STAGE_ASSISTANT_INIT == wifi_img_stage)
    {
        seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                     img2wifi[0],
                                                     XM,
                                                     YM);
        wifi_img_stage = WIFI_IMG_STAGE_READY;
    }
}

void wifi_img_send(void)
{
    if(WIFI_IMG_STAGE_READY != wifi_img_stage)
    {
        return;
    }

    seekfree_assistant_camera_send();
}
