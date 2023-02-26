#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/spi_master.h>
#include "st77903_driver.h"
#include "st77903_init_para.h"


const char *TAG = "st77903 driver";

spi_device_handle_t qspi_lcd_dev;
st77903_lcd_desc_t lcd_desc;
uint8_t **frame_buffer = NULL;

void lcdqspi_clear(uint32_t color) {
    for (int i = 0; i < LCD_Y_SIZE; ++i) {
        memset(frame_buffer, (int) color, sizeof(color));
    }
}

static void IRAM_ATTR lcdqspi_te_isr(void *args) {

}

static void lcdqspi_transmit(uint8_t cmd, int len, const uint8_t *data) {
    esp_err_t ret;
    spi_transaction_t lcd_msg;

    lcd_msg.length = (len) * 8;
    lcd_msg.rx_buffer = NULL;

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
        lcd_msg.rxlength = 0;
        lcd_msg.tx_buffer = data;

        ret = spi_device_transmit(qspi_lcd_dev, &lcd_msg);
        ESP_ERROR_CHECK(ret);
    } else {                           /* write display data by hbyte length */
        lcd_msg.flags = SPI_TRANS_MODE_QIO;
        lcd_msg.cmd = 0xde;
        lcd_msg.addr = cmd << 8ULL;
        lcd_msg.tx_buffer = data;

        ret = spi_device_transmit(qspi_lcd_dev, &lcd_msg);
        //todo: use interrupt to transmit
        ESP_ERROR_CHECK(ret);
    }
}

_Noreturn static void lcd_frame_flush(void *args) {
    // alloc lcd frame buffer in heap
    frame_buffer = (uint8_t **) malloc(LCD_Y_SIZE * sizeof(uint8_t));

    for (int i = 0; i < LCD_Y_SIZE; ++i) {
        frame_buffer[i] = (uint8_t *) malloc(LCD_HBYTE * sizeof(uint8_t));
    }

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
            lcdqspi_transmit(0x60, LCD_HBYTE, (uint8_t *) &frame_buffer[i][0]);
        }

        /* hfp(0x60) packet */
        for (int i = 0; i < LCD_HFP; i++) {
            lcdqspi_transmit(0x60, 0, NULL);
            esp_rom_delay_us(40);
        }

        /* transmit completed, can update frame cache in blanking time */
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    for (int i = 0; i < LCD_Y_SIZE; ++i) {
        if (frame_buffer[i] != NULL) {
            free(frame_buffer[i]);
            frame_buffer[i] = NULL;
        }
    }

    if (frame_buffer != NULL) {
        free(frame_buffer);
        frame_buffer = NULL;
    }
}

void lcdqspi_initialize(st77903_lcd_desc_t *lcd_init_desc) {
    esp_err_t ret;

    memcpy(&lcd_desc, lcd_init_desc, sizeof(st77903_lcd_desc_t));

    spi_bus_config_t buscfg = {
            .sclk_io_num = lcd_desc.clk_pin,
            .data0_io_num = lcd_desc.d0_pin,
            .data1_io_num = lcd_desc.d1_pin,
            .data2_io_num = lcd_desc.d2_pin,
            .data3_io_num = lcd_desc.d3_pin,
            .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD,
            .max_transfer_sz = 1200,
    };

    spi_device_interface_config_t devcfg = {
            .command_bits = 8,
            .address_bits = 24,
            .dummy_bits = 0,
            .clock_speed_hz = lcd_desc.spi_freq,
            .mode=0,                                //SPI mode 0 CPOL=0 CPHA
            .spics_io_num=lcd_desc.cs_pin,
            .queue_size=3,
            .flags = SPI_DEVICE_HALFDUPLEX,
    };

    ESP_LOGI(TAG, "Initializing QSPI Bus.");
    ret = spi_bus_initialize(lcd_desc.spi_bus, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Initializing QSPI Device.");
    ret = spi_bus_add_device(lcd_desc.spi_bus, &devcfg, &qspi_lcd_dev);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing none QSPI GPIOs.");
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1U << lcd_desc.rst_pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    ret = gpio_config(&io_conf);
    ESP_ERROR_CHECK(ret);

    if (lcd_desc.te_pin != -1) {
        io_conf.pin_bit_mask = (1U << lcd_desc.te_pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        ret = gpio_config(&io_conf);
        ESP_ERROR_CHECK(ret);
        ret = gpio_intr_enable(lcd_desc.te_pin);
        ESP_ERROR_CHECK(ret);
        //install gpio isr service
        ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        ESP_ERROR_CHECK(ret);
        //hook isr handler for specific gpio pin
        ret = gpio_isr_handler_add(lcd_desc.te_pin, lcdqspi_te_isr,
                             NULL);
        ESP_ERROR_CHECK(ret);
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
        ret = ledc_timer_config(&ledc_timer);
        ESP_ERROR_CHECK(ret);

        ledc_channel_config_t ledc_channel = {
                .channel    = lcd_desc.bl_channel,
                .duty       = 0,
                .gpio_num   = lcd_desc.bl_pin,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .hpoint     = 0,
                .timer_sel  = lcd_desc.bl_timer,
                .flags.output_invert = 0
        };

        ret = ledc_channel_config(&ledc_channel);
        ESP_ERROR_CHECK(ret);
    } else {
        ESP_LOGW(TAG,
                 "backlight pwm timer or channel not configured, initializing as gpio pin.");

        io_conf.pin_bit_mask = (1U << lcd_desc.bl_pin);
        io_conf.mode = GPIO_MODE_OUTPUT;
        ret = gpio_config(&io_conf);
        ESP_ERROR_CHECK(ret);
    }

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

    // after written init parameter, turn on backlight
    if (lcd_desc.bl_timer != -1 && lcd_desc.bl_channel != -1) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, lcd_desc.bl_channel, 255);
    } else {
        gpio_set_level(lcd_desc.bl_pin, 1);
    }

    xTaskCreatePinnedToCore(lcd_frame_flush, "lcd flush", 1024, NULL,
                            CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT, NULL,
                            lcd_desc.core_pinned);
}

