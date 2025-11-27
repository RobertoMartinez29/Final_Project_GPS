#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "neo6m.h"
#include "2d_velocity.h"
#include "esp_log.h"

static const char *TAG_MAIN = "MAIN";

static void gps_task(void *arg)
{
    ESP_LOGI(TAG_MAIN, "Starting GPS task");
    gps_start();

    while (1) {
        // 1) Read latest NMEA into neo6m's internal buffer
        raw_nmea();

        // 2) Get 2-D ground velocity
        float v_north = 0.0f;
        float v_east  = 0.0f;

        if (gps_get_ground_velocity_ms(&v_north, &v_east)) {
            // Print only what you care about:
            ESP_LOGI(TAG_MAIN,
                     "v_north=%.2f m/s, v_east=%.2f m/s",
                     v_north, v_east);
        }

        // Optional: small delay if you want to reduce log rate
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    xTaskCreatePinnedToCore(
        gps_task,
        "gps_task",
        4096,
        NULL,
        5,
        NULL,
        1
    );
}
