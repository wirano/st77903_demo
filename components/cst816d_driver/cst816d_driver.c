#include "cst816d_driver.h"
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "cst816d_driver"

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

cst816d_tp_desc_t _cst816d_desc;

void cst816d_tp_read(cst816d_tp_data_t *data) {
    uint8_t rx_data[6];
    uint8_t tx_data;

    tx_data = 0x01;
    i2c_master_write_read_device(_cst816d_desc.i2c_port, CST816D_ADDRESS,
                                 &tx_data,
                                 sizeof(tx_data), rx_data,
                                 sizeof(rx_data),
                                 pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    data->gesture = rx_data[0];
    data->finger_num = rx_data[1];
    data->event = rx_data[2] >> 6U;
    data->pos_x = (((rx_data[2] & 0x0f) << 8U) | rx_data[3]);
    data->pos_y = ((rx_data[4] << 8U) | rx_data[5]);

//    ESP_LOG_BUFFER_HEX(TAG, data, sizeof(cst816d_tp_data_t));
    ESP_LOGI(TAG, "event:%d, pos_x:%d, pos_y:%d", data->event, data->pos_x,
             data->pos_y);
}

void cst816d_tp_init(cst816d_tp_desc_t *desc) {
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
    if (desc->rst_io_num != -1) {
        io_conf.pin_bit_mask = (1ULL << desc->rst_io_num);
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.intr_type = GPIO_INTR_DISABLE;

        ESP_ERROR_CHECK(gpio_config(&io_conf));

        gpio_set_level(desc->rst_io_num, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(desc->rst_io_num, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(desc->rst_io_num, 1);
    }

    if (desc->int_io_num != -1) {
        io_conf.pin_bit_mask = (1ULL << desc->int_io_num);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }

    memcpy(&_cst816d_desc, desc, sizeof(_cst816d_desc));
}
