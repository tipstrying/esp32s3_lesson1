/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// #include "../components/lvgl/demos/lv_demos.h"
#include <inttypes.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "fontx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "sdkconfig.h"
#include "ws2812.h"

static void flush_cb(lv_disp_t *drv, const lv_area_t *area,
                     lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
  // copy a buffer's content to a specific area of the display
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1,
                            offsety2 + 1, color_map);
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver
 * parameters are updated. */
static void example_lvgl_port_update_callback(lv_disp_t *drv) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;

  esp_lcd_panel_swap_xy(panel_handle, false);
  esp_lcd_panel_mirror(panel_handle, true, false);
}
static void example_increase_lvgl_tick(void *arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(10);
}

lv_disp_t *lv_tft_espi_create(uint32_t hor_res, uint32_t ver_res, void *buf,
                              uint32_t buf_size_bytes) {
  esp_lcd_panel_io_spi_config_t io_config = {
      .dc_gpio_num = 14,
      .cs_gpio_num = 10,
      .pclk_hz = (20 * 1000 * 1000),
      .spi_mode = 0,
      .trans_queue_depth = 10,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .on_color_trans_done = NULL,
      .user_ctx = NULL,
  };

  esp_lcd_panel_io_handle_t io_handle = NULL;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,
                                           &io_config, &io_handle));
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = -1,
      .rgb_endian = LCD_RGB_ENDIAN_RGB,
      .bits_per_pixel = 16,
  };
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

  // user can flush pre-defined pattern to the screen before we turn on the
  // screen or backlight
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

  gpio_set_level(9, 1);
  lv_disp_t *disp = lv_disp_create(hor_res, ver_res);
  if (disp == NULL) {
    return NULL;
  }

  lv_disp_set_flush_cb(disp, flush_cb);
  lv_disp_set_draw_buffers(disp, (void *)buf, NULL, buf_size_bytes,
                           LV_DISP_RENDER_MODE_PARTIAL);
  return disp;
}

void app_main(void) {
  printf("Hello world!\n");

  /* Print chip information */
  esp_chip_info_t chip_info;
  uint32_t flash_size;
  esp_chip_info(&chip_info);

  /*
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ", "esp32s3",
         chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

  unsigned major_rev = chip_info.revision / 100;
  unsigned minor_rev = chip_info.revision % 100;
  printf("silicon revision v%d.%d, ", major_rev, minor_rev);
  if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    printf("Get flash size failed");
    return;
  }

  printf(
      "%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
      (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

  printf("Minimum free heap size: %" PRIu32 " bytes\n",
         esp_get_minimum_free_heap_size());

  fflush(stdout);
  */

  // contains internal graphic buffer(s) called draw buffer(s)
  static lv_color_t disp_buf;

  static lv_disp_t disp_drv;  // contains callback functions

  printf("Turn off LCD backlight");
  gpio_config_t bk_gpio_config = {.mode = GPIO_MODE_OUTPUT,
                                  .pin_bit_mask = 1ULL << 9};
  ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

  spi_bus_config_t buscfg = {.sclk_io_num = 12,
                             .mosi_io_num = 11,
                             .miso_io_num = -1,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1,
                             .max_transfer_sz = 320 * 128 * sizeof(uint16_t)};

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // spi_master_init(&tft1, 11, 12, 10, 14, -1, 9);

  lv_init();
  lv_color_t *buf1 =
      heap_caps_malloc(320 * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lv_disp_t *disp =
      lv_tft_espi_create(320, 120, buf1, 320 * 20 * sizeof(lv_color_t));

  assert(buf1);

  ESP_LOGI("Register display driver to LVGL");
  lv_disp_drv_init(&disp);

  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  ESP_LOGI("Install LVGL tick timer");
  // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &example_increase_lvgl_tick, .name = "lvgl_tick"};
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 10 * 1000));

  ws2812_control_init();

  struct led_state new_state;
  new_state.leds[0] = RED;

  ws2812_write_leds(new_state);

  //  lcdDrawString(&tft1, NULL, 0, 0, (uint8_t *)"test", 0xf800);

  /*A static or global variable to store the buffers*/

  for (;;) {
    vTaskDelay(500);
    new_state.leds[0] = GREEN;
    ws2812_write_leds(new_state);
    vTaskDelay(500);
    new_state.leds[0] = YELLOW;
    ws2812_write_leds(new_state);
    vTaskDelay(500);
    new_state.leds[0] = BLUE;
    ws2812_write_leds(new_state);
  }
  esp_restart();
}
