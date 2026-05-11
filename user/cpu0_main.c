#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

#include "imu.h"
#include "motor.h"
#include "use_img.h"
#include "wifi_img.h"
#include "zf_device_ips200.h"

#define USE_IMU_MODULE 0

int core0_main(void)
{
    // System init
    clock_init();
    debug_init();
    disable_Watchdog();
    system_delay_init();

    // DMA capture and 10ms control ISR depend on global interrupts
    interrupt_global_enable(0);

    // Display init
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();

    // Core modules
    use_img_init();
    wifi_img_init();
#if USE_IMU_MODULE
    imu_init();
#endif
    motor_init();

    cpu_wait_event_ready();

#if (MOTOR_TEST_MODE == 1)
    motor_encoder_test_task();
#elif (MOTOR_TEST_MODE == 2)
    motor_openloop_pwm_test_task();
#endif

    while(TRUE)
    {
        wifi_img_task();
        use_img();
        motor_display_status_task();
    }
}

#pragma section all restore
