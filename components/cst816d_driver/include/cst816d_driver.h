#include <esp_err.h>
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"

#define I2C_MASTER_TIMEOUT_MS 100
#define CST816D_ADDRESS 0x15

typedef struct {
    i2c_port_t i2c_port;
    int i2c_freq;

    gpio_num_t sda_io_num;
    gpio_num_t scl_io_num;
    gpio_num_t int_io_num;
    gpio_num_t rst_io_num;
} cst816d_tp_desc_t;

typedef enum {
    NONE,
    SLID_UP,
    SLID_DOWN,
    SLID_LEFT,
    SLID_RIGHT,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    LONG_PUSH
} cst816d_gesture_t;

typedef struct {
    uint8_t gesture;
    uint8_t finger_num;
    uint8_t event;   // Event (0 = Down, 1 = Up, 2 = Contact)
    uint16_t pos_x;
    uint16_t pos_y;
} cst816d_tp_data_t;

void cst816d_tp_init(cst816d_tp_desc_t *desc);

void cst816d_tp_read(cst816d_tp_data_t *data);
