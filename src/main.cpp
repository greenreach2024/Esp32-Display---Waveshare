#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_rom_sys.h"

#include "waveshare_lcd_port.h"
#include "scd40_sensor.h"

extern "C" void app_main(void)
{
    static const char *TAG = "AppMain";

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_rom_printf("\n[AppMain] Booting LCD diagnostic firmware...\n");

    esp_chip_info_t chip_info = {};
    esp_chip_info(&chip_info);
    ESP_LOGI(
        TAG,
        "Chip model=%d cores=%d features=0x%08x revision=%d IDF=%s",
        static_cast<int>(chip_info.model),
        chip_info.cores,
        static_cast<unsigned>(chip_info.features),
        chip_info.revision,
        esp_get_idf_version()
    );

    ESP_LOGI(TAG, "Starting Waveshare LCD pin diagnostic");

    const esp_err_t pin_err = waveshare_lcd_pin_test();
    if (pin_err != ESP_OK) {
        ESP_LOGW(TAG, "Pin test failed or produced no visible output: %s", esp_err_to_name(pin_err));
    } else {
        ESP_LOGI(TAG, "Pin test completed successfully. Update the pin map based on the successful candidate.");
    }

    const esp_err_t init_err = waveshare_lcd_init();
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(init_err));
        return;
    }

    constexpr float mock_temp_c = 23.7f;
    constexpr float mock_humidity = 45.2f;
    constexpr int mock_co2_ppm = 612;
    const esp_err_t dashboard_err = waveshare_lcd_draw_environment_dashboard(mock_temp_c, mock_humidity, mock_co2_ppm);
    if (dashboard_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to render environment dashboard: %s", esp_err_to_name(dashboard_err));
    } else {
        ESP_LOGI(
            TAG,
            "Dashboard rendered with mock data (T=%.1fC H=%.1f%% CO2=%dppm)",
            static_cast<double>(mock_temp_c),
            static_cast<double>(mock_humidity),
            mock_co2_ppm
        );
    }

    const esp_err_t sensor_err = scd40_sensor_start();
    if (sensor_err != ESP_OK) {
        ESP_LOGE(TAG, "SCD40 sensor start failed: %s", esp_err_to_name(sensor_err));
        while (true) {
            ESP_LOGI(TAG, "IDLE loop -- LCD showing mock data (sensor unavailable)");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    int last_valid_co2_ppm = mock_co2_ppm;
    bool have_valid_co2 = false;

    ESP_LOGI(TAG, "Waiting for live SCD40 readings...");
    while (true) {
        Scd40Reading reading{};
        const esp_err_t next_err = scd40_sensor_wait_for_reading(&reading, portMAX_DELAY);
        if ((next_err == ESP_OK) && reading.valid) {
            ESP_LOGI(
                TAG,
                "Sample #%lu CO2=%d ppm Temp=%.2f C Humidity=%.2f %%",
                static_cast<unsigned long>(reading.sample_index),
                reading.co2_ppm,
                reading.temperature_c,
                reading.humidity_percent
            );

            // Use live CO2 if plausible (>0), otherwise keep last known good value
            int co2_for_display = reading.co2_ppm;
            if (reading.co2_ppm > 0) {
                last_valid_co2_ppm = reading.co2_ppm;
                have_valid_co2 = true;
            } else {
                if (have_valid_co2) {
                    ESP_LOGW(TAG, "Sensor reported CO2=0, using last valid value: %d ppm", last_valid_co2_ppm);
                    co2_for_display = last_valid_co2_ppm;
                } else {
                    ESP_LOGW(TAG, "Sensor reported CO2=0, no valid reading yet, using mock: %d ppm", mock_co2_ppm);
                    co2_for_display = mock_co2_ppm;
                }
            }

            const esp_err_t refresh_err = waveshare_lcd_draw_environment_dashboard(
                reading.temperature_c,
                reading.humidity_percent,
                co2_for_display
            );
            if (refresh_err != ESP_OK) {
                ESP_LOGW(TAG, "Dashboard refresh failed: %s", esp_err_to_name(refresh_err));
            }
        } else {
            ESP_LOGW(TAG, "Timeout waiting for SCD40 reading: %s", esp_err_to_name(next_err));
        }
    }
}
