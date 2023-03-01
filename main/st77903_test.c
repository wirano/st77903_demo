#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "st77903_driver.h"
#include "lv_port.h"


#define TAG "main"

void app_main(void) {
    lvgl_init();

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
