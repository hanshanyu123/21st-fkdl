#ifndef CODE_WIFI_IMG_H_
#define CODE_WIFI_IMG_H_

#include "zf_common_headfile.h"

#define WIFI_IMG_ENABLE      (1)
#define WIFI_SSID_TEST        "x"
#define WIFI_PASSWORD_TEST    "sqyfunny17"

void wifi_img_init(void);
void wifi_img_task(void);
void wifi_img_send(void);

#endif
