#include "waveshare_lcd_port.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
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
static void destroy_lcd_instance(LCD *lcd);
LCDPtr g_active_lcd(nullptr, destroy_lcd_instance);
std::unique_ptr<esp_expander::CH422G> g_board_expander;

static bool expander_write_pin(int pin, uint8_t value)
{
    if (!g_board_expander) {
        ESP_LOGW(TAG, "Attempted to drive pin %d before expander init", pin);
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
    ESP_LOGI(TAG, "Configuring board idle levels via CH422G expander");
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

    ESP_LOGI(TAG, "Pulsing LCD reset through CH422G");
    if (!expander_write_pin(LCD_RST, LOW)) {
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!expander_write_pin(LCD_RST, HIGH)) {
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_LOGI(TAG, "LCD reset pulse complete");
    return ESP_OK;
}

static esp_err_t ensure_io_expander_ready(void)
{
    if (g_board_expander) {
        ESP_LOGI(TAG, "Reusing previously initialized CH422G expander");
        configure_board_idle_levels();
        return ESP_OK;
    }

    ESP_LOGI(
        TAG,
        "Initializing CH422G expander (SCL=%d SDA=%d addr=0x%02X)",
        I2C_MASTER_SCL_IO,
        I2C_MASTER_SDA_IO,
        IO_EXPANDER_CH422G_ADDRESS
    );
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
    ESP_LOGI(TAG, "CH422G expander ready");
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

static uint16_t to_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static void fill_rect(
    std::vector<uint16_t> &frame, int screen_width, int screen_height,
    int x, int y, int w, int h, uint16_t color)
{
    const int x0 = std::clamp(x, 0, screen_width);
    const int y0 = std::clamp(y, 0, screen_height);
    const int x1 = std::clamp(x + w, 0, screen_width);
    const int y1 = std::clamp(y + h, 0, screen_height);
    if ((x0 >= x1) || (y0 >= y1)) {
        return;
    }

    for (int row = y0; row < y1; ++row) {
        auto *row_ptr = frame.data() + static_cast<size_t>(row) * screen_width;
        std::fill(row_ptr + x0, row_ptr + x1, color);
    }
}

struct Glyph {
    char ch;
    std::array<uint8_t, 7> rows;
};

static constexpr std::array<Glyph, 42> FONT_5X7 = {{{'0', {0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110}},
    {'1', {0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
    {'2', {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111}},
    {'3', {0b01110, 0b10001, 0b00001, 0b00110, 0b00001, 0b10001, 0b01110}},
    {'4', {0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010}},
    {'5', {0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110}},
    {'6', {0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110}},
    {'7', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000}},
    {'8', {0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110}},
    {'9', {0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100}},
    {'A', {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
    {'B', {0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110}},
    {'C', {0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110}},
    {'D', {0b11100, 0b10010, 0b10001, 0b10001, 0b10001, 0b10010, 0b11100}},
    {'E', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111}},
    {'F', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000}},
    {'G', {0b01110, 0b10001, 0b10000, 0b10111, 0b10001, 0b10001, 0b01110}},
    {'H', {0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
    {'I', {0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
    {'J', {0b00111, 0b00010, 0b00010, 0b00010, 0b10010, 0b10010, 0b01100}},
    {'K', {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001}},
    {'L', {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111}},
    {'M', {0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001}},
    {'N', {0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001}},
    {'O', {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
    {'P', {0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000}},
    {'Q', {0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101}},
    {'R', {0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001}},
    {'S', {0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110}},
    {'T', {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100}},
    {'U', {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
    {'V', {0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b01010, 0b00100}},
    {'W', {0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b10101, 0b01010}},
    {'X', {0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001}},
    {'Y', {0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100}},
    {'Z', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111}},
    {' ', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}},
    {'.', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00110, 0b00110}},
    {'-', {0b00000, 0b00000, 0b00000, 0b01110, 0b00000, 0b00000, 0b00000}},
    {'%', {0b11001, 0b11001, 0b00010, 0b00100, 0b01000, 0b10011, 0b10011}},
    {'/', {0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b00000, 0b00000}},
    {':', {0b00000, 0b00110, 0b00110, 0b00000, 0b00110, 0b00110, 0b00000}}}};

static const Glyph *find_glyph(char c)
{
    for (const auto &glyph : FONT_5X7) {
        if (glyph.ch == c) {
            return &glyph;
        }
    }
    return nullptr;
}

static void draw_scaled_pixel_block(
    std::vector<uint16_t> &frame, int screen_width, int screen_height,
    int x, int y, int scale, uint16_t color)
{
    for (int dy = 0; dy < scale; ++dy) {
        const int py = y + dy;
        if ((py < 0) || (py >= screen_height)) {
            continue;
        }
        for (int dx = 0; dx < scale; ++dx) {
            const int px = x + dx;
            if ((px < 0) || (px >= screen_width)) {
                continue;
            }
            frame[static_cast<size_t>(py) * screen_width + px] = color;
        }
    }
}

static int text_pixel_width(const std::string &text, int scale, int letter_spacing)
{
    const int space_width = 3 * scale;
    int max_width = 0;
    int line_width = 0;
    const int spacing = letter_spacing * scale;
    for (char ch : text) {
        if (ch == '\n') {
            max_width = std::max(max_width, line_width);
            line_width = 0;
            continue;
        }
        if (ch == ' ') {
            line_width += space_width + spacing;
            continue;
        }
        const char normalized = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
        const Glyph *glyph = find_glyph(normalized);
        const int glyph_width = (glyph != nullptr) ? (5 * scale) : (5 * scale);
        line_width += glyph_width + spacing;
    }
    if (line_width > 0) {
        line_width -= spacing;
    }
    max_width = std::max(max_width, line_width);
    return max_width;
}

static void draw_text(
    std::vector<uint16_t> &frame, int screen_width, int screen_height,
    int x, int y, const std::string &text, uint16_t color,
    int scale, int letter_spacing = 1, int line_spacing = 2)
{
    int cursor_x = x;
    int cursor_y = y;
    const int spacing = letter_spacing * scale;
    const int newline_advance = (7 * scale) + (line_spacing * scale);
    for (char raw_ch : text) {
        if (raw_ch == '\n') {
            cursor_x = x;
            cursor_y += newline_advance;
            continue;
        }
        if (raw_ch == ' ') {
            cursor_x += (3 * scale) + spacing;
            continue;
        }
        const char ch = static_cast<char>(std::toupper(static_cast<unsigned char>(raw_ch)));
        const Glyph *glyph = find_glyph(ch);
        if (glyph == nullptr) {
            cursor_x += (5 * scale) + spacing;
            continue;
        }
        for (int row = 0; row < 7; ++row) {
            for (int col = 0; col < 5; ++col) {
                if (glyph->rows[row] & (1 << (4 - col))) {
                    draw_scaled_pixel_block(frame, screen_width, screen_height,
                        cursor_x + col * scale, cursor_y + row * scale, scale, color);
                }
            }
        }
        cursor_x += (5 * scale) + spacing;
    }
}

struct Rect {
    int x;
    int y;
    int w;
    int h;
};

struct MetricCardData {
    std::string title;
    std::string value;
    std::string unit;
    uint16_t accent_color;
};

static std::string format_float_value(float value, int decimals)
{
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, static_cast<double>(value));
    return std::string(buffer);
}

static std::string format_int_value(int value)
{
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%d", value);
    return std::string(buffer);
}

static void draw_background_grid(
    std::vector<uint16_t> &frame, int screen_width, int screen_height,
    uint16_t background_color, uint16_t grid_color)
{
    fill_rect(frame, screen_width, screen_height, 0, 0, screen_width, screen_height, background_color);

    constexpr int grid_step = 32;
    for (int y = 0; y < screen_height; y += grid_step) {
        fill_rect(frame, screen_width, screen_height, 0, y, screen_width, 1, grid_color);
    }

    for (int x = 0; x < screen_width; x += grid_step) {
        fill_rect(frame, screen_width, screen_height, x, 0, 1, screen_height, grid_color);
    }
}

static void draw_metric_card(
    std::vector<uint16_t> &frame, int screen_width, int screen_height,
    const Rect &card, const MetricCardData &metric,
    uint16_t card_background, uint16_t primary_text_color, uint16_t secondary_text_color)
{
    // Card body and accent strip
    fill_rect(frame, screen_width, screen_height, card.x, card.y, card.w, card.h, card_background);
    fill_rect(frame, screen_width, screen_height, card.x, card.y, card.w, 6, metric.accent_color);
    fill_rect(frame, screen_width, screen_height, card.x, card.y + card.h - 6, card.w, 6, metric.accent_color);

    const int padding = 16;
    const int title_scale = 3;
    const int value_scale = 8;
    const int unit_scale = 4;

    const int title_x = card.x + padding;
    const int title_y = card.y + padding + 4;
    draw_text(frame, screen_width, screen_height, title_x, title_y, metric.title, secondary_text_color, title_scale);

    const int value_width = text_pixel_width(metric.value, value_scale, 1);
    const int value_x = card.x + std::max(0, (card.w - value_width) / 2);
    const int value_y = card.y + (card.h / 2) - ((7 * value_scale) / 2);
    draw_text(frame, screen_width, screen_height, value_x, value_y, metric.value, primary_text_color, value_scale);

    const int unit_width = text_pixel_width(metric.unit, unit_scale, 1);
    const int unit_x = card.x + std::max(0, (card.w - unit_width) / 2);
    const int unit_y = card.y + card.h - (unit_scale * 7) - padding;
    draw_text(frame, screen_width, screen_height, unit_x, unit_y, metric.unit, metric.accent_color, unit_scale);
}

static bool draw_digit_two_pattern(LCD *lcd)
{
    const int width = lcd->getFrameWidth();
    const int height = lcd->getFrameHeight();
    if ((width <= 0) || (height <= 0)) {
        ESP_LOGE(TAG, "Invalid LCD dimensions (%d x %d)", width, height);
        return false;
    }

    ESP_LOGI(TAG, "Drawing digit 2 pattern (%dx%d, RGB565)", width, height);

    const uint16_t bg = to_rgb565(0x00, 0x05, 0x08);
    const uint16_t fg = to_rgb565(0xFF, 0xE4, 0x3C);
    std::vector<uint16_t> frame(static_cast<size_t>(width) * height, bg);

    const int margin = std::max(10, std::min(width, height) / 12);
    const int seg = std::max(8, std::min(width, height) / 24);
    const int digit_left = margin;
    const int digit_top = margin;
    const int digit_width = width - (2 * margin);
    const int digit_height = height - (2 * margin);
    if ((digit_width <= 0) || (digit_height <= 0)) {
        ESP_LOGE(TAG, "Not enough space to draw digit 2");
        return false;
    }

    const int digit_right = digit_left + digit_width;
    const int digit_bottom = digit_top + digit_height;
    const int digit_mid = digit_top + (digit_height / 2);

    // Segment layout for the digit 2 (seven-segment style: A, B, G, E, D)
    fill_rect(frame, width, height, digit_left, digit_top, digit_width, seg, fg); // Segment A
    fill_rect(frame, width, height, digit_right - seg, digit_top, seg, digit_height / 2 - (seg / 2), fg); // Segment B
    fill_rect(frame, width, height, digit_left, digit_mid - (seg / 2), digit_width, seg, fg); // Segment G
    fill_rect(frame, width, height, digit_left, digit_mid + (seg / 2), seg, digit_height / 2 - seg, fg); // Segment E
    fill_rect(frame, width, height, digit_left, digit_bottom - seg, digit_width, seg, fg); // Segment D

    // Slightly chamfer lower transitions for a smoother "2"
    fill_rect(frame, width, height, digit_right - (2 * seg), digit_mid + seg, seg * 2, seg, fg);
    fill_rect(frame, width, height, digit_left, digit_mid, seg * 2, seg, fg);

    return lcd->drawBitmap(0, 0, width, height, reinterpret_cast<uint8_t *>(frame.data()), -1);
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

#if EXAMPLE_LCD_RGB_DISABLE_FRAMEBUFFER
    ESP_LOGI(TAG, "Disabling RGB frame buffer to conserve internal RAM");
    if (!bus->configRGB_NoFrameBuffer(true)) {
        ESP_LOGW(TAG, "RGB bus refused frame buffer disable request");
    }
#endif

    ESP_LOGI(TAG, "Configuring RGB bounce buffer (%d px)", EXAMPLE_LCD_RGB_BOUNCE_BUFFER_SIZE);
    if (!bus->configRGB_BounceBufferSize(EXAMPLE_LCD_RGB_BOUNCE_BUFFER_SIZE)) {
        ESP_LOGW(TAG, "Configuring RGB bounce buffer size failed");
    }

    ESP_LOGI(TAG, "Configuring RGB timing flags (pclk_active_neg=%d)", static_cast<int>(pclk_active_neg));
    if (!bus->configRGB_TimingFlags(
            EXAMPLE_LCD_RGB_TIMING_HSYNC_IDLE_LOW,
            EXAMPLE_LCD_RGB_TIMING_VSYNC_IDLE_LOW,
            EXAMPLE_LCD_RGB_TIMING_DE_IDLE_HIGH,
            pclk_active_neg,
            EXAMPLE_LCD_RGB_TIMING_PCLK_IDLE_HIGH)) {
        ESP_LOGW(TAG, "Configuring RGB timing flags failed");
    }

    ESP_LOGI(TAG, "Calling lcd->init()");
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

    ESP_LOGI(TAG, "Issuing lcd->reset()");
    if (!lcd->reset()) {
        ESP_LOGE(TAG, "LCD reset failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Issuing lcd->begin()");
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
    g_active_lcd.reset(create_lcd_with_config());
#else
    ESP_LOGI(TAG, "Initializing RGB LCD with default config");
    g_active_lcd.reset(create_lcd_without_config());
#endif

    LCD *lcd = g_active_lcd.get();
    if (lcd == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate LCD instance");
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t init_err = initialize_lcd_common(lcd, EXAMPLE_LCD_RGB_TIMING_PCLK_ACTIVE_NEG);
    if (init_err != ESP_OK) {
        g_active_lcd.reset();
        return init_err;
    }

    ESP_LOGI(TAG, "Drawing RGB color bar test pattern");
    if (!lcd->colorBarTest()) {
        ESP_LOGE(TAG, "Color bar test failed");
        g_active_lcd.reset();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RGB LCD initialized and color bar rendered");
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

esp_err_t waveshare_lcd_draw_digit(int digit)
{
    LCD *lcd = g_active_lcd.get();
    if (lcd == nullptr) {
        ESP_LOGE(TAG, "Cannot draw digit because LCD is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (digit != 2) {
        ESP_LOGW(TAG, "Digit %d not implemented; only digit 2 is supported for now", digit);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "Attempting to draw digit %d", digit);
    if (!draw_digit_two_pattern(lcd)) {
        ESP_LOGE(TAG, "Failed to draw digit 2 pattern");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Rendered digit %d pattern on LCD", digit);
    return ESP_OK;
}

esp_err_t waveshare_lcd_draw_environment_dashboard(float temperature_c, float humidity_percent, int co2_ppm)
{
    LCD *lcd = g_active_lcd.get();
    if (lcd == nullptr) {
        ESP_LOGE(TAG, "Cannot draw dashboard because LCD is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    const int width = lcd->getFrameWidth();
    const int height = lcd->getFrameHeight();
    if ((width <= 0) || (height <= 0)) {
        ESP_LOGE(TAG, "Invalid LCD dimensions for dashboard (%d x %d)", width, height);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(
        TAG,
        "Drawing environment dashboard (temp=%.2fC humidity=%.2f%% co2=%d ppm)",
        static_cast<double>(temperature_c),
        static_cast<double>(humidity_percent),
        co2_ppm
    );

    std::vector<uint16_t> frame(static_cast<size_t>(width) * height, 0);
    const uint16_t background = to_rgb565(0x08, 0x15, 0x24);
    const uint16_t grid = to_rgb565(0x10, 0x24, 0x36);
    const uint16_t card_background = to_rgb565(0x12, 0x24, 0x38);
    const uint16_t text_primary = to_rgb565(0xF0, 0xF6, 0xFF);
    const uint16_t text_secondary = to_rgb565(0xA0, 0xC0, 0xD8);

    draw_background_grid(frame, width, height, background, grid);

    const int header_margin_x = 40;
    const int header_top = 24;
    draw_text(frame, width, height, header_margin_x, header_top, "ENVIRONMENT DASHBOARD", text_primary, 5);
    draw_text(
        frame,
        width,
        height,
        header_margin_x,
        header_top + 60,
        "LIVE SENSOR DATA",
        text_secondary,
        3
    );

    const int margin = 28;
    const int card_spacing = 18;
    const int header_height = 120;
    const int footer_height = 80;
    const int cards_height = height - header_height - footer_height - margin;
    const int card_width = (width - (2 * margin) - (2 * card_spacing)) / 3;
    if ((cards_height <= 0) || (card_width <= 0)) {
        ESP_LOGE(TAG, "Dashboard layout collapsed (cards_height=%d card_width=%d)", cards_height, card_width);
        return ESP_ERR_INVALID_SIZE;
    }

    const int cards_top = header_height;
    const int co2_display = std::max(co2_ppm, 0);
    const std::array<MetricCardData, 3> cards = {{
        MetricCardData{"TEMPERATURE", format_float_value(temperature_c, 1), "C", to_rgb565(0xFF, 0x8A, 0x3C)},
        MetricCardData{"HUMIDITY", format_float_value(humidity_percent, 1), "%RH", to_rgb565(0x4C, 0xD9, 0x92)},
        MetricCardData{"CO2", format_int_value(co2_display), "PPM", to_rgb565(0x66, 0xAD, 0xFF)},
    }};

    for (size_t idx = 0; idx < cards.size(); ++idx) {
        Rect card_rect{
            margin + static_cast<int>(idx) * (card_width + card_spacing),
            cards_top,
            card_width,
            cards_height,
        };

        // simple drop shadow for depth
        const uint16_t shadow = to_rgb565(0x07, 0x10, 0x18);
        fill_rect(frame, width, height, card_rect.x + 6, card_rect.y + 8, card_rect.w, card_rect.h, shadow);
        draw_metric_card(frame, width, height, card_rect, cards[idx], card_background, text_primary, text_secondary);
    }

    const int footer_y = height - footer_height;
    const uint16_t footer_bg = to_rgb565(0x0E, 0x1C, 0x2A);
    fill_rect(frame, width, height, margin, footer_y, width - (2 * margin), footer_height - 20, footer_bg);
    draw_text(
        frame,
        width,
        height,
        margin + 16,
        footer_y + 16,
        "SENSOR FEED: LIVE DATA",
        text_primary,
        3
    );
    draw_text(
        frame,
        width,
        height,
        margin + 16,
        footer_y + 48,
        "REPLACE WITH LIVE READINGS FROM TEMP/HUM/CO2 DRIVERS",
        text_secondary,
        2
    );

    if (!lcd->drawBitmap(0, 0, width, height, reinterpret_cast<uint8_t *>(frame.data()), -1)) {
        ESP_LOGE(TAG, "LCD rejected dashboard frame upload");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Environment dashboard rendered successfully");
    return ESP_OK;
}
