#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos//semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_async_memcpy.h"
#include "st77903_driver.h"
#include "st77903_init_para.h"


static const char *TAG = "st77903_driver";

async_memcpy_t async_flush = NULL;
SemaphoreHandle_t flush_async_sem = NULL;

spi_device_handle_t qspi_lcd_dev;
st77903_lcd_desc_t lcd_desc;
//EXT_RAM_BSS_ATTR uint16_t frame_buffer[LCD_Y_SIZE][LCD_HBYTE / 2];
#if LCD_BPP == 16
EXT_RAM_BSS_ATTR uint16_t frame_buffer[LCD_Y_SIZE][LCD_X_SIZE];
#endif

static void _async_flush_init(void);

static IRAM_ATTR bool
lcdqspi_async_memcpy_cb(async_memcpy_t mcp_hdl, async_memcpy_event_t *event,
                        void *cb_args);

static void _qspi_init(void);

static void _gpio_init(void);

static void _write_para(void);

void lcdqspi_fill_block(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1,
                        uint32_t color) {
#if LCD_BPP == 24
    if ((x1 < LCD_X_SIZE) && (y1 < LCD_Y_SIZE) && (x1 >= x0) && (y1 >= y0)) {
        uint32_t x, y;
        for (y = y0; y <= y1; y++) {
            uint32_t oft = x0 * 3;
            for (x = x0; x <= x1; x++) {
                frame_buffer[y][oft + 2] = color & 0xff;
                frame_buffer[y][oft + 1] = (color & 0xff00) >> 8;
                frame_buffer[y][oft + 0] = (color & 0xff0000) >> 16;
                oft += 3;
            }
        }
    }
#elif LCD_BPP == 16
    if ((x1 < LCD_X_SIZE) && (y1 < LCD_Y_SIZE) && (x1 >= x0) && (y1 >= y0)) {
        uint32_t x, y;
        for (y = y0; y <= y1; y++) {
            memset(&frame_buffer[y][x0], color, (x1 - x0 + 1) * LCD_HBYTE);
        }
    }
#endif
}

void lcdqspi_clear(uint32_t color) {
#if LCD_BPP == 24
    lcdqspi_fill_block(0, 0, LCD_X_SIZE - 1, LCD_Y_SIZE - 1, color);
#elif LCD_BPP == 16
//    memset(frame_buffer, color, LCD_HBYTE * LCD_X_SIZE);
    lcdqspi_fill_block(0, 0, LCD_X_SIZE - 1, LCD_Y_SIZE - 1, color);
#endif
}

void lcdqspi_draw_line(uint32_t x0, uint32_t x1, uint32_t y, uint32_t *pixel) {
    if ((x1 < LCD_X_SIZE) && (y < LCD_Y_SIZE) && (x1 >= x0)) {
#if LCD_BPP == 24
        uint32_t oft = x0 * 3;
        uint32_t *dat = pixel;
        for (uint32_t x = x0; x <= x1; x++) {
            frame_buffer[y][oft + 2] = *dat & 0xff;
            frame_buffer[y][oft + 1] = (*dat & 0xff00) >> 8;
            frame_buffer[y][oft + 0] = (*dat & 0xff0000) >> 16;
            oft += 3;
            dat++;
        }
#elif LCD_BPP == 16
        memcpy(&frame_buffer[y][x0], (uint16_t *) pixel,
               LCD_PBYTE * (x1 - x0 + 1));
//        ESP_ERROR_CHECK(esp_async_memcpy(async_flush, frame_buffer[y] + x0,
//                                         (uint16_t *) pixel,
//                                         LCD_PBYTE * (x1 - x0 + 1),
//                                         lcdqspi_async_memcpy_cb,
//                                         flush_async_sem));
//        xSemaphoreTake(flush_async_sem,portMAX_DELAY);
#endif
    }
}

static void IRAM_ATTR lcdqspi_te_isr(void *args) {

}

// Callback implementation, running in ISR context
static IRAM_ATTR bool
lcdqspi_async_memcpy_cb(async_memcpy_t mcp_hdl, async_memcpy_event_t *event,
                        void *cb_args) {
    SemaphoreHandle_t sem = (SemaphoreHandle_t) cb_args;
    BaseType_t high_task_wakeup = pdFALSE;
    xSemaphoreGiveFromISR(flush_async_sem,
                          &high_task_wakeup); // high_task_wakeup set to pdTRUE if some high priority task unblocked
    return high_task_wakeup == pdTRUE;
}


void lcdspi_fill_block_async(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1,
                             uint32_t *color) {
    ESP_ERROR_CHECK(
            esp_async_memcpy(async_flush, frame_buffer[y0], color,
                             (x1 - x0) * (y1 - y0),
                             lcdqspi_async_memcpy_cb, flush_async_sem));
    // Wait until the buffer copy is done
    xSemaphoreTake(flush_async_sem, portMAX_DELAY);
}

static void lcdqspi_transmit(uint8_t cmd, int len, const uint8_t *data) {
    esp_err_t ret;
    spi_transaction_t lcd_msg;

    lcd_msg.length = (len) * 8;
    lcd_msg.flags = 0;
    lcd_msg.rx_buffer = NULL;
    lcd_msg.rxlength = 0;

    if (len == 0) {                           /* write command, no parameter */
        lcd_msg.cmd = 0xde;
        lcd_msg.addr = cmd << 8ULL;
        lcd_msg.tx_buffer = NULL;

        ret = spi_device_transmit(qspi_lcd_dev, &lcd_msg);
        ESP_ERROR_CHECK(ret);
    } else if (len <=
               CMD_DATA_LEN_MAX) {                           /* write command with parameter */
        lcd_msg.cmd = 0xde;
        lcd_msg.addr = cmd << 8ULL;
        lcd_msg.tx_buffer = data;

        ret = spi_device_transmit(qspi_lcd_dev, &lcd_msg);
        ESP_ERROR_CHECK(ret);
    } else {                           /* write display data by hbyte length */
        lcd_msg.flags = SPI_TRANS_MODE_QIO;
        lcd_msg.cmd = 0xde;
        lcd_msg.addr = cmd << 8ULL;
        lcd_msg.tx_buffer = data;

        ret = spi_device_transmit(qspi_lcd_dev, &lcd_msg);
        ESP_ERROR_CHECK(ret);
    }
}

_Noreturn static void lcd_frame_flush(void *args) {
    // alloc lcd frame buffer in heap
//    frame_buffer = (uint8_t **) heap_caps_malloc(LCD_Y_SIZE * sizeof(uint8_t *),
//                                                 MALLOC_CAP_DMA|MALLOC_CAP_SPIRAM);
//
//    for (int i = 0; i < LCD_Y_SIZE; ++i) {
//        frame_buffer[i] = (uint8_t *) heap_caps_malloc(
//                LCD_HBYTE * sizeof(uint8_t), MALLOC_CAP_DMA|MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
//    }

//    lcdqspi_clear(0x00ffff);

    while (1) {
        /* vs(0x61) packet */
        for (int i = 0; i < LCD_VSW; i++) {
            lcdqspi_transmit(0x61, 0, NULL);
            esp_rom_delay_us(40);
        }

        /* hbp(0x60) packet */
        for (int i = 0; i < LCD_HBP; i++) {
            lcdqspi_transmit(0x60, 0, NULL);
            esp_rom_delay_us(40);
        }

        /* transmit display cache data to lcd line by line */

        for (int i = 0; i < LCD_Y_SIZE; i++) {
            lcdqspi_transmit(0x60, LCD_HBYTE, (uint8_t *) frame_buffer[i]);
        }

        /* hfp(0x60) packet */
        for (int i = 0; i < LCD_HFP; i++) {
            lcdqspi_transmit(0x60, 0, NULL);
            esp_rom_delay_us(40);
        }

        /* transmit completed, can update frame cache in blanking time */
        vTaskDelay(15 / portTICK_PERIOD_MS);
    }

//    for (int i = 0; i < LCD_Y_SIZE; ++i) {
//        if (frame_buffer[i] != NULL) {
//            free(frame_buffer[i]);
//            frame_buffer[i] = NULL;
//        }
//    }
//
//    if (frame_buffer != NULL) {
//        free(frame_buffer);
//        frame_buffer = NULL;
//    }

    vTaskDelete(NULL);
}

static void _async_flush_init(void) {

    async_memcpy_config_t async_copy_config = ASYNC_MEMCPY_DEFAULT_CONFIG();
    // update the maximum data stream supported by underlying DMA engine
    async_copy_config.backlog = 50;
    async_copy_config.sram_trans_align = 16;
    async_copy_config.psram_trans_align = 16;
    ESP_ERROR_CHECK(esp_async_memcpy_install(&async_copy_config,
                                             &async_flush)); // install driver, return driver handle
    flush_async_sem = xSemaphoreCreateBinary();
}

static void _qspi_init(void) {
    spi_bus_config_t buscfg = {
            .sclk_io_num = lcd_desc.clk_pin,
            .data0_io_num = lcd_desc.d0_pin,
            .data1_io_num = lcd_desc.d1_pin,
            .data2_io_num = lcd_desc.d2_pin,
            .data3_io_num = lcd_desc.d3_pin,
            .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD,
            .max_transfer_sz = 2048,
    };

    spi_device_interface_config_t devcfg = {
            .command_bits = 8,
            .address_bits = 24,
            .clock_speed_hz = lcd_desc.spi_freq,
            .mode=0,                                //SPI mode 0 CPOL=0 CPHA=0
            .spics_io_num=lcd_desc.cs_pin,
            .queue_size=3,
            .flags = SPI_DEVICE_HALFDUPLEX,
    };

    ESP_LOGI(TAG, "Initializing QSPI Bus.");
    ESP_ERROR_CHECK(
            spi_bus_initialize(lcd_desc.spi_bus, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "Initializing QSPI Device.");
    ESP_ERROR_CHECK(
            spi_bus_add_device(lcd_desc.spi_bus, &devcfg, &qspi_lcd_dev));
}

void _gpio_init(void) {
    ESP_LOGI(TAG, "Initializing none QSPI GPIOs.");
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1U << lcd_desc.rst_pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if (lcd_desc.te_pin != -1) {
        io_conf.pin_bit_mask = (1ULL << lcd_desc.te_pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        ESP_ERROR_CHECK(gpio_intr_enable(lcd_desc.te_pin));
        //install gpio isr service
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
        //hook isr handler for specific gpio pin
        ESP_ERROR_CHECK(gpio_isr_handler_add(lcd_desc.te_pin, lcdqspi_te_isr,
                                             NULL));
    } else {
        ESP_LOGW(TAG, "TE pin is not configured.");
    }


    if (lcd_desc.bl_timer != -1 && lcd_desc.bl_channel != -1) {
        /*
         * Prepare and set configuration of timers
         * that will be used by LED Controller
         */
        ledc_timer_config_t ledc_timer = {
                .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
                .freq_hz = 50 *
                           1000,                     // frequency of PWM signal
                .speed_mode = LEDC_LOW_SPEED_MODE,   // timer mode
                .timer_num = lcd_desc.bl_timer,      // timer index
                .clk_cfg = LEDC_AUTO_CLK,            // Auto select the source clock
        };
        // Set configuration of timer0 for high speed channels
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        ledc_channel_config_t ledc_channel = {
                .channel    = lcd_desc.bl_channel,
                .duty       = 0,
                .gpio_num   = lcd_desc.bl_pin,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .hpoint     = 0,
                .timer_sel  = lcd_desc.bl_timer,
                .flags.output_invert = 0
        };

        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    } else {
        ESP_LOGW(TAG,
                 "backlight pwm timer or channel not configured, initializing as gpio pin.");

        io_conf.pin_bit_mask = (1ULL << lcd_desc.bl_pin);
        io_conf.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
}

static void _write_para(void) {
    ESP_LOGI(TAG, "Writing LCD parameters.");
    gpio_set_level(lcd_desc.rst_pin, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(lcd_desc.rst_pin, 1);
    vTaskDelay(120 / portTICK_PERIOD_MS);

    for (int i = 0; i < sizeof(st77903_init_parameter) /
                        sizeof(st77903_init_para_t); ++i) {
        if (st77903_init_parameter[i].cmd == 0xff) {
            vTaskDelay(st77903_init_parameter[i].data[0] / portTICK_PERIOD_MS);
        } else {
            lcdqspi_transmit(st77903_init_parameter[i].cmd,
                             st77903_init_parameter[i].len,
                             (const uint8_t *) st77903_init_parameter[i].data);
        }
    }
}

void lcdqspi_initialize(st77903_lcd_desc_t *lcd_init_desc) {
    memcpy(&lcd_desc, lcd_init_desc, sizeof(st77903_lcd_desc_t));

    _qspi_init();

    _gpio_init();

    _write_para();

//    _async_flush_init();

    // after written init parameter, turn on backlight
    if (lcd_desc.bl_timer != -1 && lcd_desc.bl_channel != -1) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, lcd_desc.bl_channel, 255);
    } else {
        gpio_set_level(lcd_desc.bl_pin, 1);
    }


    xTaskCreatePinnedToCore(lcd_frame_flush, "lcd flush", 4096, NULL,
                            CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT, NULL,
                            lcd_desc.core_pinned);
}
