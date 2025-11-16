#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "waveshare_lcd_port.h"

extern "C" void app_main(void)
{
    static const char *TAG = "AppMain";

    ESP_LOGI(TAG, "Starting Waveshare LCD pin diagnostic");

    const esp_err_t pin_err = waveshare_lcd_pin_test();
    if (pin_err != ESP_OK) {
        ESP_LOGW(TAG, "Pin test failed, falling back to default init: %s", esp_err_to_name(pin_err));

        const esp_err_t err = waveshare_lcd_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(err));
            return;
        }

        ESP_LOGI(TAG, "RGB LCD default configuration initialized");
    } else {
        ESP_LOGI(TAG, "Pin test completed successfully. Update the pin map based on the successful candidate.");
    }

    while (true) {
        ESP_LOGI(TAG, "IDLE loop");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
