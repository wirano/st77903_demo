#include "cst816d_driver.h"
#include <string.h>
#include "driver/i2c.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"

#define TAG "cst816d_driver"

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

RingbufHandle_t _cst816d_data_rb = NULL;
SemaphoreHandle_t _cst816d_wait_sem = NULL;
cst816d_tp_desc_t _cst816d_desc;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

static IRAM_ATTR void cst816d_int_isr(void *args) {
    xSemaphoreGiveFromISR(_cst816d_wait_sem, NULL);
}

static void _cst816d_get_data(cst816d_data_t *data) {
    uint8_t rx_data[6];
    uint8_t tx_data;

    tx_data = 0x01;
//    i2c_master_write_to_device(I2C_NUM_0, CST816D_ADDRESS, &tx_data, 1,
//                               I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
//    vTaskDelay(pdMS_TO_TICKS(50));
//    i2c_master_read_from_device(I2C_NUM_0, CST816D_ADDRESS, rx_data, 6,
//                                I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_master_write_read_device(_cst816d_desc.i2c_port, CST816D_ADDRESS,
                                 &tx_data,
                                 sizeof(tx_data), rx_data, sizeof(rx_data),
                                 pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    ESP_LOG_BUFFER_HEX(TAG, rx_data, 6);
//    vTaskDelay(pdMS_TO_TICKS(50));
    data->gesture = rx_data[0];
    data->finger_num = rx_data[1];
    data->x[0] = rx_data[2];
    data->x[1] = rx_data[3];
    data->y[0] = rx_data[4];
    data->y[1] = rx_data[5];

    ESP_LOG_BUFFER_HEX(TAG,data, sizeof(cst816d_data_t));
    ESP_LOG_BUFFER_HEX(TAG,rx_data, sizeof(rx_data));
}

static void cst816d_task(void *args) {
    cst816d_data_t data;

    while (1) {
        if (_cst816d_desc.int_io_num != 0 && _cst816d_desc.int_io_num != -1) {
            _cst816d_get_data(&data);
            xRingbufferSend(_cst816d_data_rb, &data, sizeof(data),
                            pdMS_TO_TICKS(50));
            xSemaphoreTake(_cst816d_wait_sem, portMAX_DELAY);
        } else {
            _cst816d_get_data(&data);
            if (data.finger_num > 0) {
                xRingbufferSend(_cst816d_data_rb, &data, sizeof(data),
                                pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    vTaskDelete(NULL);
}

void cs816d_tp_read(cst816d_data_t *data, int timeout) {
    data = xRingbufferReceive(_cst816d_data_rb, sizeof(cst816d_data_t),
                              pdMS_TO_TICKS(timeout));
}

void cst816d_tp_init(cst816d_tp_desc_t *desc) {
    _cst816d_wait_sem = xSemaphoreCreateBinary();
    // Ringbuffer requires an additional 8 bytes for a header
    _cst816d_data_rb = xRingbufferCreate((sizeof(cst816d_data_t) + 8) * 16,
                                         RINGBUF_TYPE_NOSPLIT);

    i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = desc->sda_io_num,
            .scl_io_num = desc->scl_io_num,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = desc->i2c_freq
    };

    ESP_ERROR_CHECK(i2c_param_config(desc->i2c_port, &i2c_conf));

    ESP_ERROR_CHECK(i2c_driver_install(desc->i2c_port, I2C_MODE_MASTER,
                                       I2C_MASTER_RX_BUF_DISABLE,
                                       I2C_MASTER_TX_BUF_DISABLE,
                                       0));


    gpio_config_t io_conf;
    if (desc->rst_io_num != -1 && desc->rst_io_num != 0) {
        io_conf.pin_bit_mask = (1ULL << desc->rst_io_num);
        io_conf.mode = GPIO_MODE_OUTPUT;

        ESP_ERROR_CHECK(gpio_config(&io_conf));

        gpio_set_level(desc->rst_io_num, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(desc->rst_io_num, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(desc->rst_io_num, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (desc->int_io_num != -1 && desc->int_io_num != 0) {
        io_conf.pin_bit_mask = (1ULL << desc->int_io_num);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        ESP_ERROR_CHECK(gpio_intr_enable(desc->int_io_num));
        //install gpio isr service
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
        //hook isr handler for specific gpio pin
        ESP_ERROR_CHECK(gpio_isr_handler_add(desc->int_io_num, cst816d_int_isr,
                                             NULL));
    }

    memcpy(&_cst816d_desc, desc, sizeof(_cst816d_desc));

    xTaskCreatePinnedToCore(cst816d_task, "cst816d_tp_task", portSTACK_GROWTH,
                            NULL, CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT, NULL,
                            1);
}

#pragma clang diagnostic pop