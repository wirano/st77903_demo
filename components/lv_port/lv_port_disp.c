/**
 * @file lv_port_disp_templ.c
 *
 */


/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include "st77903_driver.h"

/*********************
 *      DEFINES
 *********************/
#define MY_DISP_HOR_RES        400
#define MY_DISP_VER_RES        400


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

#ifndef MY_DISP_HOR_RES
    #warning Please define or replace the macro MY_DISP_HOR_RES with the actual screen width, default value 320 is used for now.
    #define MY_DISP_HOR_RES    320
#endif

#ifndef MY_DISP_VER_RES
    #warning Please define or replace the macro MY_DISP_HOR_RES with the actual screen height, default value 240 is used for now.
    #define MY_DISP_VER_RES    240
#endif


static void disp_init(void);

static void disp_flush(lv_disp_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

void lv_port_disp_init(void)
{
    disp_init();

    lv_disp_t * disp = lv_disp_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);
    static lv_color_t buf_1[MY_DISP_HOR_RES * 10];                          /*A buffer for 10 rows*/
    lv_disp_set_draw_buffers(disp, buf_1, NULL, MY_DISP_HOR_RES * 10,LV_DISP_RENDER_MODE_PARTIAL);   /*Initialize the display buffer*/
    lv_disp_set_flush_cb(disp, disp_flush);
    lv_disp_set_color_format(disp,LV_COLOR_FORMAT_RGB888);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    st77903_lcd_desc_t lcd = {
            .spi_bus = SPI2_HOST,
            .cs_pin = LCD_PIN_CS,
            .d0_pin = LCD_PIN_D0,
            .d1_pin = LCD_PIN_D1,
            .d2_pin = LCD_PIN_D2,
            .d3_pin = LCD_PIN_D3,
            .spi_freq = 40 * 1000 * 1000,
            .clk_pin = LCD_PIN_CLK,
            .te_pin = -1,
            .bl_pin = LCD_PIN_BLK,
            .bl_timer = -2
    };

    lcdqspi_initialize(&lcd);
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(disp_flush_enabled) {
        /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/

        int32_t x;
        int32_t y;
        for(y = area->y1; y <= area->y2; y++) {
            for(x = area->x1; x <= area->x2; x++) {
                /*Put a pixel to the display. For example:*/
                /*put_px(x, y, *color_p)*/
                color_p++;
            }
        }
    }

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
