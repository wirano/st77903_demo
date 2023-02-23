#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>

#include "st77903_init_para.h"

#define LCD_PIN_CS      10
#define LCD_PIN_CLK     12
#define LCD_PIN_D0      11
#define LCD_PIN_D1      13
#define LCD_PIN_D2      14
#define LCD_PIN_D3      9

#define LCD_PIN_RST     3
#define LCD_PIN_ID      17
#define LCD_PIN_SDA     18
#define LCD_PIN_DC      8
#define LCD_PIN_BLK     46

#define LCD_BPP             (24)

#define LCD_X_SIZE          (400U)                      /* available x pixel size */
#define LCD_Y_SIZE          (400U)                        /* available y pixle size */
#define LCD_PBYTE           ((LCD_BPP + 7) / 8)         /* bytes in pixel unit */
#define LCD_HBYTE           (LCD_X_SIZE * LCD_PBYTE)    /* bytes in horizontal line */

#define LCD_VSW         1U
#define LCD_HFP         8U
#define LCD_HBP         8U

EXT_RAM_BSS_ATTR uint8_t frame_cache[LCD_Y_SIZE][LCD_HBYTE];

const char *TAG = "main";

spi_device_handle_t lcd_dev;


void lcd_transmit(uint8_t cmd, int len, const uint8_t *data) {
    esp_err_t ret;
    spi_transaction_t lcd_msg;

    lcd_msg.length = (len) * 8;
    lcd_msg.rx_buffer = NULL;

    if (len == 0) {                           /* write command, no parameter */
        lcd_msg.cmd = 0xde;
        lcd_msg.addr = cmd << 8ULL;
        lcd_msg.tx_buffer = NULL;

        ret = spi_device_transmit(lcd_dev, &lcd_msg);
        ESP_ERROR_CHECK(ret);
    } else if (len <= DATA_LEN_MAX) {                           /* write command with parameter */
        lcd_msg.cmd = 0xde;
        lcd_msg.addr = cmd << 8ULL;
        lcd_msg.rxlength = 0;
        lcd_msg.tx_buffer = data;

        ret = spi_device_transmit(lcd_dev, &lcd_msg);
        ESP_ERROR_CHECK(ret);
    } else {                           /* write display data by hbyte length */
        lcd_msg.flags = SPI_TRANS_MODE_QIO;
        lcd_msg.cmd = 0xde;
        lcd_msg.addr = cmd << 8ULL;
        lcd_msg.tx_buffer = data;

        ret = spi_device_transmit(lcd_dev, &lcd_msg);
        ESP_ERROR_CHECK(ret);
    }
}

void lcd_initialize(void) {
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(110 / portTICK_PERIOD_MS);

    for (int i = 0; i < sizeof(st77903_init_parameter) / sizeof(st77903_init_para_t); ++i) {
        if (st77903_init_parameter[i].cmd == 0xff) {
            vTaskDelay(st77903_init_parameter[i].data[0] / portTICK_PERIOD_MS);
        } else {
            lcd_transmit(st77903_init_parameter[i].cmd, st77903_init_parameter[i].len,
                         (const uint8_t *) st77903_init_parameter[i].data);
        }
    }
}

_Noreturn static void lcd_frame_flush(void *args) {
    while (1) {
        /* vs(0x61) packet */
        for (int i = 0; i < LCD_VSW; i++) {
            lcd_transmit(0x61, 0, NULL);
            esp_rom_delay_us(40);
        }

        /* hbp(0x60) packet */
        for (int i = 0; i < LCD_HBP; i++) {
            lcd_transmit(0x60, 0, NULL);
            esp_rom_delay_us(40);
        }

        /* transmit display cache data to lcd line by line */

        for (int i = 0; i < LCD_Y_SIZE; i++) {
            lcd_transmit(0x60, LCD_HBYTE, (uint8_t *) &frame_cache[i][0]);
        }

        /* hfp(0x60) packet */
        for (int i = 0; i < LCD_HFP; i++) {
            lcd_transmit(0x60, 0, NULL);
            esp_rom_delay_us(40);
        }

        /* transmit is complet, can update frame cache in blanking time */
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void lcdqspi_set_pixel(uint32_t x, uint32_t y, uint32_t color) {
    if ((x < LCD_X_SIZE) && (y < LCD_Y_SIZE)) {
        uint32_t xoft = x * 3;
        frame_cache[y][xoft + 2] = color & 0xff;
        frame_cache[y][xoft + 1] = (color & 0xff00) >> 8;
        frame_cache[y][xoft + 0] = (color & 0xff0000) >> 16;
    }
}

void lcdqspi_fill_block(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, uint32_t color) {
    if ((x1 < LCD_X_SIZE) && (y1 < LCD_Y_SIZE) && (x1 >= x0) && (y1 >= y0)) {
        uint32_t x, y;
        for (y = y0; y <= y1; y++) {
            uint32_t oft = x0 * 3;
            for (x = x0; x <= x1; x++) {
                frame_cache[y][oft + 2] = color & 0xff;
                frame_cache[y][oft + 1] = (color & 0xff00) >> 8;
                frame_cache[y][oft + 0] = (color & 0xff0000) >> 16;
                oft += 3;
            }
        }
    }
}

void lcdqspi_clear(uint32_t color) {
    lcdqspi_fill_block(0, 0, LCD_X_SIZE - 1, LCD_Y_SIZE - 1, color);
}

void lcdqspi_draw_line(uint32_t x0, uint32_t x1, uint32_t y, uint32_t *pixel) {
    if ((x1 < LCD_X_SIZE) && (y < LCD_Y_SIZE) && (x1 >= x0)) {
        uint32_t oft = x0 * 3;
        uint32_t *dat = pixel;
        for (uint32_t x = x0; x <= x1; x++) {
            frame_cache[y][oft + 2] = *dat & 0xff;
            frame_cache[y][oft + 1] = (*dat & 0xff00) >> 8;
            frame_cache[y][oft + 0] = (*dat & 0xff0000) >> 16;
            oft += 3;
            dat++;
        }
    }
}

void lcdqspi_draw_line_rbg888(uint32_t x0, uint32_t x1, uint32_t y, uint8_t *dat) {
    if ((x1 < LCD_X_SIZE) && (y < LCD_Y_SIZE) && (x1 >= x0)) {
        uint32_t oft = x0 * 3;
        for (uint32_t x = x0; x <= x1; x++) {
            frame_cache[y][oft + 2] = *dat++;
            frame_cache[y][oft + 1] = *dat++;
            frame_cache[y][oft + 0] = *dat++;
            oft += 3;
        }
    }
}

void app_main(void) {
    esp_err_t ret;

    spi_bus_config_t buscfg = {
            .sclk_io_num = LCD_PIN_CLK,
            .data0_io_num = LCD_PIN_D0,
            .data1_io_num = LCD_PIN_D1,
            .data2_io_num = LCD_PIN_D2,
            .data3_io_num = LCD_PIN_D3,
            .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD,
            .max_transfer_sz = 2048,
    };

    spi_device_interface_config_t devcfg = {
            .command_bits = 8,
            .address_bits = 24,
            .dummy_bits = 0,
            .clock_speed_hz=80 * 1000 * 1000,           //Clock out at 10 MHz
            .mode=0,                                //SPI mode 0 CPOL=0 CPHA
            .spics_io_num=LCD_PIN_CS,               //CS pin
            .queue_size=3,                          //We want to be able to queue 7 transactions at a time
            .flags = SPI_DEVICE_HALFDUPLEX,
//            .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    ESP_LOGI(TAG, "Initializing QSPI Bus.");
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Initializing QSPI Device.");
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &lcd_dev);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing none QSPI GPIOs.");
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL << LCD_PIN_RST) | (1ULL << LCD_PIN_BLK));
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pull_down_en = true;
    gpio_config(&io_conf);
    gpio_set_level(LCD_PIN_BLK, 1);
    gpio_set_level(LCD_PIN_RST, 1);

    io_conf.pin_bit_mask = ((1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_SDA)) | (1ULL << LCD_PIN_ID);
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    gpio_config(&io_conf);
    gpio_set_level(LCD_PIN_DC, 1);
    gpio_set_level(LCD_PIN_SDA, 1);
    gpio_set_level(LCD_PIN_ID, 1);

    lcd_initialize();
    ESP_LOGI(TAG, "lcd initialized.");

    lcdqspi_fill_block(0, 0, 300, 300, 0x00ffff);

    xTaskCreate(lcd_frame_flush, "lcd flush", 1024, NULL, CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT, NULL);

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
