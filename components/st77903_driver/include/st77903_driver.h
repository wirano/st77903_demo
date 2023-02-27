#include <hal/spi_types.h>
#include <hal/gpio_types.h>
#include "hal/ledc_types.h"

typedef struct {
    spi_host_device_t spi_bus;
    int spi_freq;

    // GPIO pins for qspi lcd, or -1 if not used
    gpio_num_t clk_pin;
    gpio_num_t d0_pin;
    gpio_num_t d1_pin;
    gpio_num_t d2_pin;
    gpio_num_t d3_pin;
    gpio_num_t cs_pin;
    gpio_num_t rst_pin;
    gpio_num_t te_pin;
    gpio_num_t bl_pin;

    // timer for lcd backlight, or -1 if not used
    ledc_timer_t bl_timer;
    ledc_channel_t bl_channel;

    int core_pinned;
} st77903_lcd_desc_t;

void lcdqspi_initialize(st77903_lcd_desc_t *lcd_init_desc);

void lcdqspi_clear(uint32_t color);

void lcdqspi_draw_line(uint32_t x0, uint32_t x1, uint32_t y, uint32_t *pixel);

void lcdspi_fill_block_async(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1,
                             uint32_t *color);
