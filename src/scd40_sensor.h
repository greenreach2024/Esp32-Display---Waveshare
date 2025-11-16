#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "esp_err.h"

struct Scd40Reading {
    bool valid{false};
    int co2_ppm{0};
    float temperature_c{0.0f};
    float humidity_percent{0.0f};
    uint32_t sample_index{0};
    uint64_t timestamp_ms{0};
};

esp_err_t scd40_sensor_start();
esp_err_t scd40_sensor_wait_for_reading(Scd40Reading *out_reading, TickType_t ticks_to_wait);
