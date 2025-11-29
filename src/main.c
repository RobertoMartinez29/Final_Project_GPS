#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"

#include "2d_velocity.h"
#include "dps310.h"
#include "lsm9ds1_hal.h"
#include "neo6m.h"
#include "scd41.h"

// ===================== I2C / DPS310 CONFIG =====================

#define I2C_PORT I2C_NUM_0
#define SDA_GPIO 21
#define SCL_GPIO 22

static const char *TAG_MAIN = "MAIN";
static const char *TAG_BARO = "BARO";
static const char *TAG_GPS = "GPS";
static const char *TAG_SCD41 = "SCD41";

static void i2c_init(void) {
  i2c_config_t c = {.mode = I2C_MODE_MASTER,
                    .sda_io_num = SDA_GPIO,
                    .scl_io_num = SCL_GPIO,
                    .sda_pullup_en = GPIO_PULLUP_ENABLE,
                    .scl_pullup_en = GPIO_PULLUP_ENABLE,
                    .master.clk_speed = 400000};

  ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &c));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, c.mode, 0, 0, 0));
}

// ===================== BAROMETER TASK =====================

static void baro_task(void *arg) {
  dps310_t dps;

  ESP_LOGI(TAG_BARO, "Initializing DPS310...");

  // Use 0x76 if SDO is tied to GND, 0x77 if tied to VCC (your code used 0x77)
  if (dps310_init(&dps, I2C_PORT, 0x77) != ESP_OK) {
    ESP_LOGE(TAG_BARO, "Failed to initialize DPS310");
    vTaskDelete(NULL);
  }

  // 16 Hz, OSR 16, continuous T + P
  if (dps310_config(&dps, DPS310_RATE_16HZ, DPS310_OSR_16, DPS310_RATE_16HZ,
                    DPS310_OSR_16, DPS310_MODE_CONT_PT) != ESP_OK) {
    ESP_LOGE(TAG_BARO, "Failed to configure DPS310");
    vTaskDelete(NULL);
  }

  ESP_LOGI(TAG_BARO, "DPS310 configured.");

  while (1) {
    float t_c = 0.0f;
    float p_hpa = 0.0f;

    if (dps310_read(&dps, &t_c, &p_hpa) == ESP_OK) {
      // Quick barometric altitude estimate
      float alt_m = 44330.0f * (1.0f - powf(p_hpa / 1013.25f, 0.1903f));
      ESP_LOGI(TAG_BARO, "T=%.2f C  P=%.2f hPa  Alt≈%.1f m", t_c, p_hpa, alt_m);
    } else {
      ESP_LOGW(TAG_BARO, "Failed to read DPS310");
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // 0.5 s
  }
}

// ===================== GPS TASK (2-D VELOCITY) =====================

static void gps_task(void *arg) {
  ESP_LOGI(TAG_GPS, "Starting GPS task...");
  gps_start(); // Initializes UART2 for the NEO-6M

  while (1) {
    // Read latest NMEA sentence into neo6m internal buffer
    raw_nmea();

    // Get 2-D ground velocity (North/East) in m/s
    float v_north = 0.0f;
    float v_east = 0.0f;

    if (gps_get_ground_velocity_ms(&v_north, &v_east)) {
      ESP_LOGI(TAG_GPS, "v_north=%.2f m/s  v_east=%.2f m/s", v_north, v_east);
    } else {
      ESP_LOGW(TAG_GPS, "Could not parse ground velocity");
    }

    // Optional: small delay to reduce log spam
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ===================== SCD41 TASK =====================

static void scd41_task(void *arg) {
  ESP_LOGI(TAG_SCD41, "Starting SCD41 task...");

  // Initialize SCD41
  scd41_config_t scd41_cfg = SCD41_CONFIG_DEFAULT();
  // Ensure I2C port matches what we initialized in i2c_init (I2C_NUM_0)
  scd41_cfg.i2c_port = I2C_PORT;

  // Note: i2c_init() in main already installs the driver for I2C_PORT.
  // scd41_init() checks if driver is installed? No, it just stores config and
  // stops measurement. So we are good to go since i2c_init() is called in
  // app_main.

  if (scd41_init(&scd41_cfg) != ESP_OK) {
    ESP_LOGE(TAG_SCD41, "Failed to initialize SCD41");
    vTaskDelete(NULL);
  }

  // Start periodic measurement
  if (scd41_start_measurement() != ESP_OK) {
    ESP_LOGE(TAG_SCD41, "Failed to start measurement");
    vTaskDelete(NULL);
  }

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(5000)); // SCD41 updates every 5 seconds

    bool data_ready = false;
    if (scd41_data_ready(&data_ready) == ESP_OK && data_ready) {
      scd41_data_t data;
      if (scd41_read_measurement(&data) == ESP_OK) {
        ESP_LOGI(TAG_SCD41, "CO2: %u ppm, Temp: %.2f C, Hum: %.2f %%",
                 data.co2_ppm, data.temperature, data.humidity);
      } else {
        ESP_LOGW(TAG_SCD41, "Failed to read measurement");
      }
    }
  }
}

// ===================== LSM9DS1 TASK =====================

static void lsm9ds1_task(void *arg) {
  ESP_LOGI("LSM9DS1", "Starting LSM9DS1 task...");

  // Initialize LSM9DS1 (IMU + Mag)
  // Note: I2C is already initialized in app_main, but lsm9ds1_init stores the
  // port
  if (lsm9ds1_init(I2C_PORT) != ESP_OK) {
    ESP_LOGE("LSM9DS1", "Failed to initialize LSM9DS1");
    vTaskDelete(NULL);
  }

  while (1) {
    float ax, ay, az;
    float gx, gy, gz;

    // Read Accelerometer (mg)
    if (lsm9ds1_read_accel(&ax, &ay, &az) == ESP_OK) {
      ESP_LOGI("LSM9DS1", "Accel (mg): X=%.2f Y=%.2f Z=%.2f", ax, ay, az);
    }

    // Read Gyroscope (dps)
    if (lsm9ds1_read_gyro(&gx, &gy, &gz) == ESP_OK) {
      ESP_LOGI("LSM9DS1", "Gyro (dps): X=%.2f Y=%.2f Z=%.2f", gx, gy, gz);
    }

    // Read Magnetometer (gauss) - Optional, but available
    // float mx, my, mz;
    // if (lsm9ds1_read_mag(&mx, &my, &mz) == ESP_OK) {
    //   ESP_LOGI("LSM9DS1", "Mag (gauss): X=%.2f Y=%.2f Z=%.2f", mx, my, mz);
    // }

    vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz update rate
  }
}

// ===================== APP MAIN =====================

void app_main(void) {
  // Global log level if you want
  esp_log_level_set("*", ESP_LOG_INFO);

  // Init I2C before starting barometer task
  i2c_init();

  ESP_LOGI(TAG_MAIN, "Creating tasks...");

  // GPS task: uses UART2 (configured inside gps_start)
  xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 5, NULL,
                          1 // core 1 (you can change to 0 if you want)
  );

  // Barometer task: uses I2C0
  xTaskCreatePinnedToCore(baro_task, "baro_task", 4096, NULL, 5, NULL,
                          1 // same core is fine for now
  );

  // SCD41 task: uses I2C0 (same as baro)
  xTaskCreatePinnedToCore(scd41_task, "scd41_task", 4096, NULL, 5, NULL,
                          1 // same core is fine for now
  );

  // LSM9DS1 task: uses I2C0
  xTaskCreatePinnedToCore(lsm9ds1_task, "lsm9ds1_task", 4096, NULL, 5, NULL,
                          1 // same core is fine for now
  );
}
