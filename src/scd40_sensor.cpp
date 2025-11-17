#include "scd40_sensor.h"

#include <array>

#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "waveshare_lcd_port.h"

namespace {
constexpr const char *TAG = "SCD40";
constexpr uint8_t SCD40_I2C_ADDRESS = 0x62;
constexpr TickType_t I2C_TIMEOUT_TICKS = pdMS_TO_TICKS(100);
constexpr TickType_t MEASUREMENT_PERIOD_TICKS = pdMS_TO_TICKS(5000);
constexpr TickType_t INITIAL_WARMUP_TICKS = pdMS_TO_TICKS(6000);
constexpr TickType_t SENSOR_RECOVERY_PAUSE_TICKS = pdMS_TO_TICKS(500);
constexpr TickType_t FAULT_NOTICE_MIN_INTERVAL_TICKS = pdMS_TO_TICKS(2000);
constexpr int SCD40_TASK_STACK = 4096;
constexpr UBaseType_t SCD40_TASK_PRIORITY = 5;

constexpr uint16_t CMD_START_PERIODIC_MEASUREMENT = 0x21B1;
constexpr uint16_t CMD_STOP_PERIODIC_MEASUREMENT = 0x3F86;
constexpr uint16_t CMD_READ_MEASUREMENT = 0xEC05;
constexpr uint16_t CMD_GET_DATA_READY = 0xE4B8;
constexpr uint16_t CMD_REINIT = 0x3646;

constexpr size_t WORD_SIZE = 2;
constexpr size_t CRC_SIZE = 1;
constexpr uint8_t CRC8_POLY = 0x31;
constexpr uint8_t CRC8_INIT = 0xFF;
constexpr uint32_t DATA_READY_RETRY_MS = 6000;
constexpr uint32_t DATA_READY_POLL_INTERVAL_MS = 200;
constexpr uint16_t DATA_READY_VALID_MASK = 0x07FF; // per Sensirion datasheet

QueueHandle_t s_reading_queue = nullptr;
TaskHandle_t s_task_handle = nullptr;
bool s_sensor_started = false;
TickType_t s_last_fault_notice_ticks = 0;

uint8_t compute_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = CRC8_INIT;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>((crc << 1) ^ CRC8_POLY);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

esp_err_t send_command(uint16_t command)
{
    const uint8_t payload[WORD_SIZE] = {
        static_cast<uint8_t>(command >> 8),
        static_cast<uint8_t>(command & 0xFF)
    };
    return i2c_master_write_to_device(
        I2C_MASTER_NUM, SCD40_I2C_ADDRESS, payload, sizeof(payload), I2C_TIMEOUT_TICKS
    );
}

template <size_t WORD_COUNT>
esp_err_t read_words(uint16_t command, std::array<uint16_t, WORD_COUNT> &words)
{
    const uint8_t tx[WORD_SIZE] = {
        static_cast<uint8_t>(command >> 8),
        static_cast<uint8_t>(command & 0xFF)
    };

    std::array<uint8_t, WORD_COUNT * (WORD_SIZE + CRC_SIZE)> rx{};
    const esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        SCD40_I2C_ADDRESS,
        tx,
        sizeof(tx),
        rx.data(),
        rx.size(),
        I2C_TIMEOUT_TICKS
    );
    if (err != ESP_OK) {
        return err;
    }

    for (size_t i = 0; i < WORD_COUNT; ++i) {
        const size_t idx = i * (WORD_SIZE + CRC_SIZE);
        if (compute_crc8(rx.data() + idx, WORD_SIZE) != rx[idx + WORD_SIZE]) {
            ESP_LOGW(TAG, "CRC mismatch for word %zu (cmd=0x%04X)", i, command);
            return ESP_ERR_INVALID_CRC;
        }
        words[i] = static_cast<uint16_t>((rx[idx] << 8) | rx[idx + 1]);
    }

    return ESP_OK;
}

esp_err_t read_measurement(Scd40Reading &reading)
{
    std::array<uint16_t, 3> raw_words{};
    const esp_err_t err = read_words(CMD_READ_MEASUREMENT, raw_words);
    if (err != ESP_OK) {
        return err;
    }

    reading.valid = true;
    reading.co2_ppm = static_cast<int>(raw_words[0]);
    reading.temperature_c = -45.0f + 175.0f * (static_cast<float>(raw_words[1]) / 65535.0f);
    reading.humidity_percent = 100.0f * (static_cast<float>(raw_words[2]) / 65535.0f);
    reading.timestamp_ms = static_cast<uint64_t>(esp_timer_get_time() / 1000ULL);

    ESP_LOGI(
        TAG,
        "Raw words: CO2=0x%04X Temp=0x%04X RH=0x%04X",
        raw_words[0],
        raw_words[1],
        raw_words[2]
    );

    return ESP_OK;
}

esp_err_t wait_for_data_ready()
{
    const TickType_t timeout_ticks = pdMS_TO_TICKS(DATA_READY_RETRY_MS);
    const TickType_t poll_interval_ticks = pdMS_TO_TICKS(DATA_READY_POLL_INTERVAL_MS);
    TickType_t elapsed = 0;

    while (elapsed <= timeout_ticks) {
        std::array<uint16_t, 1> status{};
        const esp_err_t err = read_words(CMD_GET_DATA_READY, status);
        if (err != ESP_OK) {
            return err;
        }

        ESP_LOGD(TAG, "Data-ready status=0x%04X", status[0]);
        const bool ready = (status[0] & DATA_READY_VALID_MASK) != 0;
        if (ready) {
            return ESP_OK;
        }

        vTaskDelay(poll_interval_ticks);
        elapsed += poll_interval_ticks;
    }

    ESP_LOGW(TAG, "Timeout waiting for SCD40 data-ready flag");
    return ESP_ERR_TIMEOUT;
}

void publish_reading(const Scd40Reading &reading)
{
    if (s_reading_queue != nullptr) {
        xQueueOverwrite(s_reading_queue, &reading);
    }
}

void publish_fault_state(const char *reason, uint32_t sample_index)
{
    const TickType_t now = xTaskGetTickCount();
    if ((now - s_last_fault_notice_ticks) < FAULT_NOTICE_MIN_INTERVAL_TICKS) {
        return;
    }

    s_last_fault_notice_ticks = now;

    Scd40Reading offline{};
    offline.valid = false;
    offline.sample_index = sample_index;
    offline.timestamp_ms = static_cast<uint64_t>(esp_timer_get_time() / 1000ULL);
    publish_reading(offline);

    ESP_LOGW(TAG, "SCD40 fault notice sent (%s)", reason);
}

void restart_periodic_session(const char *reason)
{
    ESP_LOGW(TAG, "Restarting SCD40 session (%s)", reason);

    const esp_err_t stop_err = send_command(CMD_STOP_PERIODIC_MEASUREMENT);
    if (stop_err != ESP_OK) {
        ESP_LOGW(TAG, "Stop measurement command failed: %s", esp_err_to_name(stop_err));
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    const esp_err_t reinit_err = send_command(CMD_REINIT);
    if (reinit_err != ESP_OK) {
        ESP_LOGW(TAG, "Sensor reinit command failed: %s", esp_err_to_name(reinit_err));
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    const esp_err_t start_err = send_command(CMD_START_PERIODIC_MEASUREMENT);
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start periodic measurement after recovery: %s", esp_err_to_name(start_err));
    } else {
        ESP_LOGI(TAG, "SCD40 periodic measurement restarted");
    }

    vTaskDelay(INITIAL_WARMUP_TICKS + SENSOR_RECOVERY_PAUSE_TICKS);
}

void scd40_task(void *)
{
    ESP_LOGI(TAG, "SCD40 polling task started");

    if (send_command(CMD_STOP_PERIODIC_MEASUREMENT) != ESP_OK) {
        ESP_LOGW(TAG, "Unable to stop previous measurement session");
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_err_t cmd_err = send_command(CMD_START_PERIODIC_MEASUREMENT);
    if (cmd_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start periodic measurement: %s", esp_err_to_name(cmd_err));
    } else {
        ESP_LOGI(TAG, "SCD40 periodic measurement started");
    }

    vTaskDelay(INITIAL_WARMUP_TICKS);

    uint32_t sample_index = 0;
    int consecutive_failures = 0;
    while (true) {
        const esp_err_t ready_err = wait_for_data_ready();
        if (ready_err != ESP_OK) {
            ESP_LOGW(TAG, "Data-ready poll failed: %s", esp_err_to_name(ready_err));
            publish_fault_state("data-ready", sample_index);
            if (++consecutive_failures >= 5) {
                publish_fault_state("data-ready recovery", sample_index);
                restart_periodic_session("data-ready failures");
                consecutive_failures = 0;
                sample_index = 0;
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        Scd40Reading reading{};
        const esp_err_t read_err = read_measurement(reading);
        if (read_err == ESP_OK && reading.valid) {
            consecutive_failures = 0;
            s_last_fault_notice_ticks = 0;
            reading.sample_index = ++sample_index;
            publish_reading(reading);
            ESP_LOGI(
                TAG,
                "Sample #%lu -> CO2=%d ppm Temp=%.2f C Humidity=%.2f %%",
                static_cast<unsigned long>(reading.sample_index),
                reading.co2_ppm,
                reading.temperature_c,
                reading.humidity_percent
            );
        } else {
            ESP_LOGW(TAG, "Measurement read failed: %s", esp_err_to_name(read_err));
            publish_fault_state("measurement", sample_index);
            if (++consecutive_failures >= 5) {
                publish_fault_state("measurement recovery", sample_index);
                restart_periodic_session("measurement failures");
                consecutive_failures = 0;
                sample_index = 0;
            }
        }

        // Yield briefly; next loop iteration waits via status polling.
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

} // namespace

esp_err_t scd40_sensor_start()
{
    if (s_sensor_started) {
        return ESP_OK;
    }

    if (s_reading_queue == nullptr) {
        s_reading_queue = xQueueCreate(1, sizeof(Scd40Reading));
        if (s_reading_queue == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate reading queue");
            return ESP_ERR_NO_MEM;
        }
    }

    const BaseType_t task_result = xTaskCreate(
        scd40_task,
        "scd40_poll",
        SCD40_TASK_STACK,
        nullptr,
        SCD40_TASK_PRIORITY,
        &s_task_handle
    );

    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SCD40 polling task");
        vQueueDelete(s_reading_queue);
        s_reading_queue = nullptr;
        s_task_handle = nullptr;
        return ESP_ERR_NO_MEM;
    }

    s_sensor_started = true;
    return ESP_OK;
}

esp_err_t scd40_sensor_wait_for_reading(Scd40Reading *out_reading, TickType_t ticks_to_wait)
{
    if ((s_reading_queue == nullptr) || (out_reading == nullptr)) {
        return ESP_ERR_INVALID_STATE;
    }

    const BaseType_t ok = xQueueReceive(s_reading_queue, out_reading, ticks_to_wait);
    if (ok != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}
