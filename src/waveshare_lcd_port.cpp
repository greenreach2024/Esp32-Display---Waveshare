#include "waveshare_lcd_port.h"

#include <algorithm>
#include <array>
#include <optional>
#include <cassert>
#include <cstdint>
#include <memory>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bit_defs.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_io_expander.hpp"

using namespace esp_panel::drivers;

#if EXAMPLE_LCD_ENABLE_PRINT_FPS
bool onLCD_RefreshFinishCallback(void *user_data);
#endif
#if EXAMPLE_LCD_ENABLE_DRAW_FINISH_CALLBACK
bool onLCD_DrawFinishCallback(void *user_data);
#endif

namespace {
constexpr const char *TAG = "WaveshareLCD";

struct PinProbeCandidate {
    const char *name;
    int hsync;
    int vsync;
    int de;
    int pclk;
    int disp;
    bool pclk_active_neg;
    std::array<int, EXAMPLE_LCD_RGB_DATA_WIDTH> data;
};

#if EXAMPLE_LCD_RGB_DATA_WIDTH == 16
constexpr std::array<int, EXAMPLE_LCD_RGB_DATA_WIDTH> DEFAULT_DATA_PINS = {
    EXAMPLE_LCD_RGB_IO_DATA0, EXAMPLE_LCD_RGB_IO_DATA1, EXAMPLE_LCD_RGB_IO_DATA2, EXAMPLE_LCD_RGB_IO_DATA3,
    EXAMPLE_LCD_RGB_IO_DATA4, EXAMPLE_LCD_RGB_IO_DATA5, EXAMPLE_LCD_RGB_IO_DATA6, EXAMPLE_LCD_RGB_IO_DATA7,
    EXAMPLE_LCD_RGB_IO_DATA8, EXAMPLE_LCD_RGB_IO_DATA9, EXAMPLE_LCD_RGB_IO_DATA10, EXAMPLE_LCD_RGB_IO_DATA11,
    EXAMPLE_LCD_RGB_IO_DATA12, EXAMPLE_LCD_RGB_IO_DATA13, EXAMPLE_LCD_RGB_IO_DATA14, EXAMPLE_LCD_RGB_IO_DATA15,
};
constexpr std::array<const char *, EXAMPLE_LCD_RGB_DATA_WIDTH> RGB_LANE_LABELS = {
    "B0", "B1", "B2", "B3", "B4", "G0", "G1", "G2",
    "G3", "G4", "G5", "R0", "R1", "R2", "R3", "R4",
};
#elif EXAMPLE_LCD_RGB_DATA_WIDTH == 8
constexpr std::array<int, EXAMPLE_LCD_RGB_DATA_WIDTH> DEFAULT_DATA_PINS = {
    EXAMPLE_LCD_RGB_IO_DATA0, EXAMPLE_LCD_RGB_IO_DATA1, EXAMPLE_LCD_RGB_IO_DATA2, EXAMPLE_LCD_RGB_IO_DATA3,
    EXAMPLE_LCD_RGB_IO_DATA4, EXAMPLE_LCD_RGB_IO_DATA5, EXAMPLE_LCD_RGB_IO_DATA6, EXAMPLE_LCD_RGB_IO_DATA7,
};
constexpr std::array<const char *, EXAMPLE_LCD_RGB_DATA_WIDTH> RGB_LANE_LABELS = {
    "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7",
};
#else
#error "Unsupported RGB data width"
#endif

constexpr std::array<PinProbeCandidate, 5> PIN_PROBE_CANDIDATES = {{{
    .name = "Waveshare docs (DE enabled, falling-edge PCLK)",
    .hsync = 46,
    .vsync = 3,
    .de = 5,
    .pclk = 7,
    .disp = EXAMPLE_LCD_RGB_IO_DISP,
    .pclk_active_neg = true,
    .data = DEFAULT_DATA_PINS,
}, {
    .name = "DE unused (HS/VS only)",
    .hsync = 46,
    .vsync = 3,
    .de = -1,
    .pclk = 7,
    .disp = EXAMPLE_LCD_RGB_IO_DISP,
    .pclk_active_neg = true,
    .data = DEFAULT_DATA_PINS,
}, {
    .name = "Swap HSYNC/VSYNC (rare PCB rev)",
    .hsync = 3,
    .vsync = 46,
    .de = 5,
    .pclk = 7,
    .disp = EXAMPLE_LCD_RGB_IO_DISP,
    .pclk_active_neg = true,
    .data = DEFAULT_DATA_PINS,
}, {
    .name = "PCLK rising edge", // some panels sample on rising edge
    .hsync = 46,
    .vsync = 3,
    .de = 5,
    .pclk = 7,
    .disp = EXAMPLE_LCD_RGB_IO_DISP,
    .pclk_active_neg = false,
    .data = DEFAULT_DATA_PINS,
}, {
    .name = "PCLK rising edge + no DE",
    .hsync = 46,
    .vsync = 3,
    .de = -1,
    .pclk = 7,
    .disp = EXAMPLE_LCD_RGB_IO_DISP,
    .pclk_active_neg = false,
    .data = DEFAULT_DATA_PINS,
}}};

using LCDPtr = std::unique_ptr<LCD, void (*)(LCD *)>;
std::unique_ptr<esp_expander::CH422G> g_board_expander;

static bool expander_write_pin(int pin, uint8_t value)
{
    if (!g_board_expander) {
        return false;
    }
    if (!g_board_expander->digitalWrite(pin, value)) {
        ESP_LOGW(TAG, "Failed to drive CH422G pin %d", pin);
        return false;
    }
    return true;
}

static void configure_board_idle_levels(void)
{
    expander_write_pin(USB_SEL, HIGH);
    expander_write_pin(SD_CS, HIGH);
    expander_write_pin(TP_RST, HIGH);
    expander_write_pin(LCD_BL, EXAMPLE_LCD_BL_ON_LEVEL ? HIGH : LOW);
}

static esp_err_t reset_lcd_via_expander(void)
{
    if (!g_board_expander) {
        ESP_LOGW(TAG, "LCD reset skipped because IO expander is not ready");
        return ESP_ERR_INVALID_STATE;
    }

    if (!expander_write_pin(LCD_RST, LOW)) {
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!expander_write_pin(LCD_RST, HIGH)) {
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(120));
    return ESP_OK;
}

static esp_err_t ensure_io_expander_ready(void)
{
    if (g_board_expander) {
        configure_board_idle_levels();
        return ESP_OK;
    }

    auto expander = std::make_unique<esp_expander::CH422G>(
        I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, IO_EXPANDER_CH422G_ADDRESS
    );

    if (!expander->init()) {
        ESP_LOGE(TAG, "Failed to initialize CH422G I2C host");
        return ESP_FAIL;
    }

    if (!expander->begin()) {
        ESP_LOGE(TAG, "Failed to begin CH422G expander");
        return ESP_FAIL;
    }

    if (!expander->enableAllIO_Output()) {
        ESP_LOGW(TAG, "Unable to force CH422G IO lines to output mode");
    }

    g_board_expander = std::move(expander);
    configure_board_idle_levels();
    return reset_lcd_via_expander();
}

static void destroy_lcd_instance(LCD *lcd)
{
    if (lcd == nullptr) {
        return;
    }
    if (!lcd->del()) {
        ESP_LOGW(TAG, "LCD delete reported failure");
    }
    delete lcd;
}

static LCD::Config build_default_lcd_config(void)
{
    return LCD::Config{
        .device = LCD::DevicePartialConfig{
            .reset_gpio_num = EXAMPLE_LCD_RST_IO,
            .bits_per_pixel = EXAMPLE_LCD_COLOR_BITS,
        },
        .vendor = LCD::VendorPartialConfig{
            .hor_res = EXAMPLE_LCD_WIDTH,
            .ver_res = EXAMPLE_LCD_HEIGHT,
        },
    };
}

static BusRGB::Config build_default_bus_config(void)
{
    BusRGB::Config bus_config = {
        .control_panel = std::nullopt,
        .refresh_panel = BusRGB::RefreshPanelPartialConfig{}
    };

    auto &refresh = std::get<BusRGB::RefreshPanelPartialConfig>(bus_config.refresh_panel);
    refresh.pclk_hz = EXAMPLE_LCD_RGB_TIMING_FREQ_HZ;
    refresh.h_res = EXAMPLE_LCD_WIDTH;
    refresh.v_res = EXAMPLE_LCD_HEIGHT;
    refresh.hsync_pulse_width = EXAMPLE_LCD_RGB_TIMING_HPW;
    refresh.hsync_back_porch = EXAMPLE_LCD_RGB_TIMING_HBP;
    refresh.hsync_front_porch = EXAMPLE_LCD_RGB_TIMING_HFP;
    refresh.vsync_pulse_width = EXAMPLE_LCD_RGB_TIMING_VPW;
    refresh.vsync_back_porch = EXAMPLE_LCD_RGB_TIMING_VBP;
    refresh.vsync_front_porch = EXAMPLE_LCD_RGB_TIMING_VFP;
    refresh.data_width = EXAMPLE_LCD_RGB_DATA_WIDTH;
    refresh.bits_per_pixel = EXAMPLE_LCD_RGB_COLOR_BITS;
    refresh.bounce_buffer_size_px = EXAMPLE_LCD_RGB_BOUNCE_BUFFER_SIZE;
    refresh.hsync_gpio_num = EXAMPLE_LCD_RGB_IO_HSYNC;
    refresh.vsync_gpio_num = EXAMPLE_LCD_RGB_IO_VSYNC;
    refresh.de_gpio_num = EXAMPLE_LCD_RGB_IO_DE;
    refresh.pclk_gpio_num = EXAMPLE_LCD_RGB_IO_PCLK;
    refresh.disp_gpio_num = EXAMPLE_LCD_RGB_IO_DISP;
    refresh.flags_pclk_active_neg = EXAMPLE_LCD_RGB_TIMING_PCLK_ACTIVE_NEG;

    for (size_t i = 0; i < DEFAULT_DATA_PINS.size(); ++i) {
        refresh.data_gpio_nums[i] = DEFAULT_DATA_PINS[i];
    }

    return bus_config;
}
}

static LCD *create_lcd_without_config(void)
{
    BusRGB *bus = new BusRGB(
#if EXAMPLE_LCD_RGB_DATA_WIDTH == 8
        EXAMPLE_LCD_RGB_IO_DATA0, EXAMPLE_LCD_RGB_IO_DATA1, EXAMPLE_LCD_RGB_IO_DATA2, EXAMPLE_LCD_RGB_IO_DATA3,
        EXAMPLE_LCD_RGB_IO_DATA4, EXAMPLE_LCD_RGB_IO_DATA5, EXAMPLE_LCD_RGB_IO_DATA6, EXAMPLE_LCD_RGB_IO_DATA7,
        EXAMPLE_LCD_RGB_IO_HSYNC, EXAMPLE_LCD_RGB_IO_VSYNC, EXAMPLE_LCD_RGB_IO_PCLK, EXAMPLE_LCD_RGB_IO_DE,
        EXAMPLE_LCD_RGB_IO_DISP,
        EXAMPLE_LCD_RGB_TIMING_FREQ_HZ, EXAMPLE_LCD_WIDTH, EXAMPLE_LCD_HEIGHT,
        EXAMPLE_LCD_RGB_TIMING_HPW, EXAMPLE_LCD_RGB_TIMING_HBP, EXAMPLE_LCD_RGB_TIMING_HFP,
        EXAMPLE_LCD_RGB_TIMING_VPW, EXAMPLE_LCD_RGB_TIMING_VBP, EXAMPLE_LCD_RGB_TIMING_VFP
#elif EXAMPLE_LCD_RGB_DATA_WIDTH == 16
        EXAMPLE_LCD_RGB_IO_DATA0, EXAMPLE_LCD_RGB_IO_DATA1, EXAMPLE_LCD_RGB_IO_DATA2, EXAMPLE_LCD_RGB_IO_DATA3,
        EXAMPLE_LCD_RGB_IO_DATA4, EXAMPLE_LCD_RGB_IO_DATA5, EXAMPLE_LCD_RGB_IO_DATA6, EXAMPLE_LCD_RGB_IO_DATA7,
        EXAMPLE_LCD_RGB_IO_DATA8, EXAMPLE_LCD_RGB_IO_DATA9, EXAMPLE_LCD_RGB_IO_DATA10, EXAMPLE_LCD_RGB_IO_DATA11,
        EXAMPLE_LCD_RGB_IO_DATA12, EXAMPLE_LCD_RGB_IO_DATA13, EXAMPLE_LCD_RGB_IO_DATA14, EXAMPLE_LCD_RGB_IO_DATA15,
        EXAMPLE_LCD_RGB_IO_HSYNC, EXAMPLE_LCD_RGB_IO_VSYNC, EXAMPLE_LCD_RGB_IO_PCLK, EXAMPLE_LCD_RGB_IO_DE,
        EXAMPLE_LCD_RGB_IO_DISP,
        EXAMPLE_LCD_RGB_TIMING_FREQ_HZ, EXAMPLE_LCD_WIDTH, EXAMPLE_LCD_HEIGHT,
        EXAMPLE_LCD_RGB_TIMING_HPW, EXAMPLE_LCD_RGB_TIMING_HBP, EXAMPLE_LCD_RGB_TIMING_HFP,
        EXAMPLE_LCD_RGB_TIMING_VPW, EXAMPLE_LCD_RGB_TIMING_VBP, EXAMPLE_LCD_RGB_TIMING_VFP
#endif
    );

    return new EXAMPLE_LCD_CLASS(
        EXAMPLE_LCD_NAME, bus, EXAMPLE_LCD_WIDTH, EXAMPLE_LCD_HEIGHT, EXAMPLE_LCD_COLOR_BITS, EXAMPLE_LCD_RST_IO
    );
}

#if EXAMPLE_LCD_ENABLE_CREATE_WITH_CONFIG
static LCD *create_lcd_with_config(void)
{
    return new EXAMPLE_LCD_CLASS(EXAMPLE_LCD_NAME, build_default_bus_config(), build_default_lcd_config());
}
#endif

static LCD *create_lcd_from_candidate(const PinProbeCandidate &candidate)
{
    auto bus_config = build_default_bus_config();
    auto &refresh = std::get<BusRGB::RefreshPanelPartialConfig>(bus_config.refresh_panel);
    refresh.hsync_gpio_num = candidate.hsync;
    refresh.vsync_gpio_num = candidate.vsync;
    refresh.de_gpio_num = candidate.de;
    refresh.pclk_gpio_num = candidate.pclk;
    refresh.disp_gpio_num = candidate.disp;
    refresh.flags_pclk_active_neg = candidate.pclk_active_neg;
    for (size_t i = 0; i < candidate.data.size(); ++i) {
        refresh.data_gpio_nums[i] = candidate.data[i];
    }

    return new EXAMPLE_LCD_CLASS(EXAMPLE_LCD_NAME, bus_config, build_default_lcd_config());
}

static esp_err_t initialize_lcd_common(LCD *lcd, bool pclk_active_neg)
{
    if (lcd == nullptr) {
        ESP_LOGE(TAG, "LCD handle unavailable");
        return ESP_ERR_INVALID_ARG;
    }

    auto *bus = static_cast<BusRGB *>(lcd->getBus());
    if (bus == nullptr) {
        ESP_LOGE(TAG, "LCD bus handle unavailable");
        return ESP_ERR_INVALID_STATE;
    }

    if (!bus->configRGB_BounceBufferSize(EXAMPLE_LCD_RGB_BOUNCE_BUFFER_SIZE)) {
        ESP_LOGW(TAG, "Configuring RGB bounce buffer size failed");
    }

    if (!bus->configRGB_TimingFlags(
            EXAMPLE_LCD_RGB_TIMING_HSYNC_IDLE_LOW,
            EXAMPLE_LCD_RGB_TIMING_VSYNC_IDLE_LOW,
            EXAMPLE_LCD_RGB_TIMING_DE_IDLE_HIGH,
            pclk_active_neg,
            EXAMPLE_LCD_RGB_TIMING_PCLK_IDLE_HIGH)) {
        ESP_LOGW(TAG, "Configuring RGB timing flags failed");
    }

    if (!lcd->init()) {
        ESP_LOGE(TAG, "LCD init failed");
        return ESP_FAIL;
    }

#if EXAMPLE_LCD_ENABLE_PRINT_FPS
    if (!lcd->attachRefreshFinishCallback(onLCD_RefreshFinishCallback)) {
        ESP_LOGW(TAG, "Failed to attach refresh finish callback");
    }
#endif
#if EXAMPLE_LCD_ENABLE_DRAW_FINISH_CALLBACK
    if (!lcd->attachDrawBitmapFinishCallback(onLCD_DrawFinishCallback)) {
        ESP_LOGW(TAG, "Failed to attach draw finish callback");
    }
#endif

    if (!lcd->reset()) {
        ESP_LOGE(TAG, "LCD reset failed");
        return ESP_FAIL;
    }

    if (!lcd->begin()) {
        ESP_LOGE(TAG, "LCD begin failed");
        return ESP_FAIL;
    }

    if (lcd->getBasicAttributes().basic_bus_spec.isFunctionValid(LCD::BasicBusSpecification::FUNC_DISPLAY_ON_OFF)) {
        lcd->setDisplayOnOff(true);
    }

    return ESP_OK;
}

static void log_candidate_lanes(const PinProbeCandidate &candidate)
{
    for (size_t idx = 0; idx < candidate.data.size(); ++idx) {
        const char *label = (idx < RGB_LANE_LABELS.size()) ? RGB_LANE_LABELS[idx] : "DATA";
        ESP_LOGI(
            TAG, "[%s] Lane %02u -> GPIO%02d (%s)", candidate.name, static_cast<unsigned>(idx),
            candidate.data[idx], label
        );
    }
}

static bool draw_pin_lane_pattern(LCD *lcd, const PinProbeCandidate &candidate)
{
    const int color_bits = lcd->getFrameColorBits();
    if (color_bits <= 0) {
        ESP_LOGE(TAG, "Unable to query color depth");
        return false;
    }

    const int bytes_per_pixel = color_bits / 8;
    const int width = lcd->getFrameWidth();
    const int height = lcd->getFrameHeight();
    const int stripes = static_cast<int>(candidate.data.size());
    const int stripe_height = std::max(1, height / stripes);

    std::vector<uint8_t> stripe_buffer(static_cast<size_t>(width) * stripe_height * bytes_per_pixel, 0);

    for (int lane = 0; lane < stripes; ++lane) {
        const uint32_t pattern = BIT(lane);
        const size_t pixel_count = stripe_buffer.size() / bytes_per_pixel;
        for (size_t px = 0; px < pixel_count; ++px) {
            for (int byte = 0; byte < bytes_per_pixel; ++byte) {
                stripe_buffer[px * bytes_per_pixel + byte] = pattern >> (byte * 8);
            }
        }

        const int y = lane * stripe_height;
        const int h = (lane == (stripes - 1)) ? (height - y) : stripe_height;
        if (h <= 0) {
            break;
        }

        if (!lcd->drawBitmap(0, y, width, h, stripe_buffer.data(), -1)) {
            ESP_LOGE(TAG, "Failed to draw diagnostic stripe for lane %d", lane);
            return false;
        }
    }

    return true;
}

static esp_err_t run_pin_candidate(const PinProbeCandidate &candidate)
{
    esp_err_t err = ensure_io_expander_ready();
    if (err != ESP_OK) {
        return err;
    }

    err = reset_lcd_via_expander();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LCD reset via expander failed: %s", esp_err_to_name(err));
    }

    LCDPtr lcd(create_lcd_from_candidate(candidate), destroy_lcd_instance);
    if (!lcd) {
        ESP_LOGE(TAG, "Failed to allocate LCD for candidate %s", candidate.name);
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t init_err = initialize_lcd_common(lcd.get(), candidate.pclk_active_neg);
    if (init_err != ESP_OK) {
        return init_err;
    }

    log_candidate_lanes(candidate);
    ESP_LOGI(TAG, "Rendering diagnostic stripes for candidate '%s'", candidate.name);
    if (!draw_pin_lane_pattern(lcd.get(), candidate)) {
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(EXAMPLE_LCD_PIN_TEST_HOLD_TIME_MS));
    return ESP_OK;
}

#if EXAMPLE_LCD_ENABLE_PRINT_FPS
DRAM_ATTR int frame_count = 0;
DRAM_ATTR int fps = 0;
DRAM_ATTR int64_t start_time_us = 0;

IRAM_ATTR bool onLCD_RefreshFinishCallback(void *user_data)
{
    const int64_t now_us = esp_timer_get_time();
    if (start_time_us == 0) {
        start_time_us = now_us;
        return false;
    }

    frame_count++;
    if (frame_count >= EXAMPLE_LCD_PRINT_FPS_COUNT_MAX) {
        const int64_t elapsed = now_us - start_time_us;
        if (elapsed > 0) {
            fps = static_cast<int>((EXAMPLE_LCD_PRINT_FPS_COUNT_MAX * 1000000LL) / elapsed);
            esp_rom_printf("LCD FPS: %d\n", fps);
        }
        frame_count = 0;
        start_time_us = now_us;
    }

    return false;
}
#endif // EXAMPLE_LCD_ENABLE_PRINT_FPS

#if EXAMPLE_LCD_ENABLE_DRAW_FINISH_CALLBACK
IRAM_ATTR bool onLCD_DrawFinishCallback(void *user_data)
{
    esp_rom_printf("LCD draw finish callback\n");
    return false;
}
#endif

esp_err_t waveshare_lcd_init(void)
{
    esp_err_t expander_err = ensure_io_expander_ready();
    if (expander_err != ESP_OK) {
        return expander_err;
    }

    expander_err = reset_lcd_via_expander();
    if (expander_err != ESP_OK) {
        ESP_LOGW(TAG, "LCD reset via expander failed: %s", esp_err_to_name(expander_err));
    }

#if EXAMPLE_LCD_ENABLE_CREATE_WITH_CONFIG
    ESP_LOGI(TAG, "Initializing RGB LCD with explicit config");
    LCD *lcd = create_lcd_with_config();
#else
    ESP_LOGI(TAG, "Initializing RGB LCD with default config");
    LCD *lcd = create_lcd_without_config();
#endif

    if (lcd == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate LCD instance");
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t init_err = initialize_lcd_common(lcd, EXAMPLE_LCD_RGB_TIMING_PCLK_ACTIVE_NEG);
    if (init_err != ESP_OK) {
        return init_err;
    }

    ESP_LOGI(TAG, "Drawing RGB color bar test pattern");
    if (!lcd->colorBarTest()) {
        ESP_LOGE(TAG, "Color bar test failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RGB LCD initialized");
    return ESP_OK;
}

esp_err_t waveshare_lcd_pin_test(void)
{
    ESP_LOGI(
        TAG, "Starting Waveshare LCD pin sweep across %u candidate(s)",
        static_cast<unsigned>(PIN_PROBE_CANDIDATES.size())
    );

    esp_err_t result = ESP_FAIL;
    for (const auto &candidate : PIN_PROBE_CANDIDATES) {
        ESP_LOGI(TAG, "Trying candidate: %s", candidate.name);
        const esp_err_t err = run_pin_candidate(candidate);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Candidate '%s' produced a visible pattern", candidate.name);
            result = ESP_OK;
#if EXAMPLE_LCD_PIN_TEST_STOP_ON_SUCCESS
            ESP_LOGI(
                TAG,
                "Stopping after first successful candidate (set EXAMPLE_LCD_PIN_TEST_STOP_ON_SUCCESS to 0 to scan all)"
            );
            break;
#endif
        } else {
            ESP_LOGW(TAG, "Candidate '%s' failed: %s", candidate.name, esp_err_to_name(err));
        }
    }

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Pin sweep unsuccessful for all candidates");
    }

    return result;
}
