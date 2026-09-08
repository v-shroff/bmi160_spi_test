// includes
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "filter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "hal/spi_types.h"
#include "soc/gpio_num.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <sys/types.h>
// pin nums for spi
#define SCLK_PIN GPIO_NUM_23
#define MOSI_PIN GPIO_NUM_22
#define MISO_PIN GPIO_NUM_19
#define ESP_HOST SPI3_HOST
#define BMI_CS_PIN GPIO_NUM_21
// bmi160 register addresses
#define BMI_SWITCH_SPI 0x7F
#define WHO_AM_I 0x00
#define REG_PMU_STATUS 0x03   // power mode status for acc/gyr/mag
#define REG_ACC_RANGE 0x41    // accelerometer g-range select
#define REG_GYR_RANGE 0x43    // gyroscope dps-range select
#define REG_CMD 0x7E          // command register (pmu mode, softreset, etc)
#define REG_GYR_ACC_DATA 0x0C // start of gyro+accel burst read (12 bytes)
// cmd register opcodes
#define CMD_ACC_PMU_NORMAL 0x11 // acc_set_pmu_mode, nn=01 -> accel normal mode
#define CMD_GYR_PMU_NORMAL 0x15 // gyr_set_pmu_mode, nn=01 -> gyro normal mode

// range settings (see datasheet range tables)
#define ACC_RANGE_2G 0b011     // acc_range<3:0> = 0b0011 -> +/-2g
#define GYR_RANGE_500DPS 0b010 // gyr_range<2:0> = 0b010 -> +/-500 dps

// delay times after writes (has headroom over datasheet max exec times)
#define PMU_CMD_DELAY_MS 80
#define RANGE_CFG_DELAY_MS 50

// expected values
#define EXPECTED_CHIP_ID 209        // WHO_AM_I reset value (0xD1)
#define EXPECTED_PMU_BOTH_NORMAL 20 // PMU_STATUS w/ acc+gyr normal, mag suspend
// gyro acel / physics constants
#define ACC_2G_LSB_G 16384.00
#define GYR_500_LSB 65.6 // deg/sec
#define G_MS 9.81
#define CALIBRATION_READINGS 1000
#define DEG_TO_RAD 0.017453
// global
//
// default offsets : GYR | X:    3.78  Y:  -29.10  Z:   15.55    ACC | X:626.726
// Y:1459.590  Z:109.460
float gyr_x_cal = 3.78f;
float gyr_y_cal = -29.10f;
float gyr_z_cal = 15.55f;
float acc_x_cal = 626.726;
float acc_y_cal = 1459.59f;
float acc_z_cal = 109.46f;
esp_err_t ret;
spi_device_handle_t bmi160_t;

// initalize the spi bus and attach the gyro to it
void spi_initalize() {
  spi_bus_config_t bus{
      .mosi_io_num = MOSI_PIN,
      .miso_io_num = MISO_PIN,
      .sclk_io_num = SCLK_PIN,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };
  spi_device_interface_config_t bmi160{
      .command_bits = 1,
      .address_bits = 7,
      .mode = 0,
      .clock_speed_hz = 1000 * 1000 * 7,
      .spics_io_num = BMI_CS_PIN,
      .queue_size = 5,

  };

  ret = spi_bus_initialize(ESP_HOST, &bus, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  ret = spi_bus_add_device(ESP_HOST, &bmi160, &bmi160_t);
  ESP_ERROR_CHECK(ret);
}
// use to write one byte at a time to the bmi
esp_err_t spi_write(uint8_t addr, uint8_t data, spi_device_handle_t device) {
  spi_transaction_t trans{
      .flags = SPI_TRANS_USE_TXDATA,
      .cmd = 0, // 0 = write ; 1 = read
      .addr = addr,
      .length = 8,
      .tx_data = {data},
      .rx_buffer = nullptr,
  };
  ret = spi_device_polling_transmit(device, &trans);
  return ret;
}
esp_err_t spi_read(uint8_t addr, uint8_t *data, size_t trans_len,
                   spi_device_handle_t device) // address, data output buffer,
                                               // transcation length in BYTES
{
  spi_transaction_t trans{
      .cmd = 1,
      .addr = addr,
      .length = trans_len * 8,
      .tx_buffer = nullptr,
      .rx_buffer = data,

  };
  ret = spi_device_polling_transmit(device, &trans);
  return ret;
}
void setupBmi160() {
  uint8_t data[1];
  spi_read(BMI_SWITCH_SPI, data, 1, bmi160_t);
  spi_read(WHO_AM_I, data, 1, bmi160_t);
  if (data[0] != EXPECTED_CHIP_ID) {
    ESP_LOGE("setup BMI", "wrong who am i");
    abort();
  }
  spi_write(REG_CMD, CMD_ACC_PMU_NORMAL, bmi160_t);
  vTaskDelay(pdMS_TO_TICKS(PMU_CMD_DELAY_MS));
  spi_read(REG_PMU_STATUS, data, 1, bmi160_t);
  //
  spi_write(REG_CMD, CMD_GYR_PMU_NORMAL, bmi160_t);
  vTaskDelay(pdMS_TO_TICKS(PMU_CMD_DELAY_MS));
  spi_read(REG_PMU_STATUS, data, 1, bmi160_t);
  if (data[0] != 20) {
    ESP_LOGE("setup BMI", "gyro and acel not initalized");
    abort();
  }
  //
  spi_write(REG_ACC_RANGE, ACC_RANGE_2G, bmi160_t);
  vTaskDelay(pdMS_TO_TICKS(RANGE_CFG_DELAY_MS));
  spi_read(REG_ACC_RANGE, data, 1, bmi160_t);
  if (data[0] != 0b011) {
    ESP_LOGE("setup BMI", "acc_range wrong");
    abort();
  }
  //
  spi_write(REG_GYR_RANGE, GYR_RANGE_500DPS, bmi160_t);
  vTaskDelay(pdMS_TO_TICKS(RANGE_CFG_DELAY_MS));
  spi_read(REG_GYR_RANGE, data, 1, bmi160_t);
  if (data[0] != 0b010) {
    ESP_LOGE("setup BMI", "gyro_range wrong");
    abort();
  }
}
void normalize(float &value) {
  if (value > 0) {
    value = ACC_2G_LSB_G - value;
  } else
    value = -1 * (value + ACC_2G_LSB_G);
}
void calibrate_gyro(float &gyr_x_cal, float &gyr_y_cal, float &gyr_z_cal,
                    float &acc_x_cal, float &acc_y_cal, float &acc_z_cal) {
  uint8_t acc_gyro_dat[12];
  // reset all to zero
  gyr_x_cal = 0.0f;
  gyr_y_cal = 0.0f;
  gyr_z_cal = 0.0f;
  acc_x_cal = 0.0f;
  acc_y_cal = 0.0f;
  acc_z_cal = 0.0f;
  // since its small numbers, just sum them and do one division step for the
  // average value
  //  we will be assuming the accelerometer value closest to 1 is the "gravity"
  //  vector, and that the gyro readins are all 0,0,0 at startup. Probably will
  //  only use once or twice and then hardcode offsets for now.

  for (int i = 0; i < CALIBRATION_READINGS; ++i) {
    spi_read(REG_GYR_ACC_DATA, acc_gyro_dat, 12, bmi160_t);
    gyr_x_cal += (int16_t)((acc_gyro_dat[1] << 8) | acc_gyro_dat[0]);
    gyr_y_cal += (int16_t)((acc_gyro_dat[3] << 8) | acc_gyro_dat[2]);
    gyr_z_cal += (int16_t)((acc_gyro_dat[5] << 8) | acc_gyro_dat[4]);
    acc_x_cal += (int16_t)((acc_gyro_dat[7] << 8) | acc_gyro_dat[6]);
    acc_y_cal += (int16_t)((acc_gyro_dat[9] << 8) | acc_gyro_dat[8]);
    acc_z_cal += (int16_t)((acc_gyro_dat[11] << 8) | acc_gyro_dat[10]);
  }

  gyr_x_cal = -1.0f * ((1.0f / CALIBRATION_READINGS) * gyr_x_cal);
  gyr_y_cal = -1.0f * ((1.0f / CALIBRATION_READINGS) * gyr_y_cal);
  gyr_z_cal = -1.0f * ((1.0f / CALIBRATION_READINGS) * gyr_z_cal);
  //

  acc_x_cal = ((1.0f / CALIBRATION_READINGS) * acc_x_cal);
  acc_y_cal = ((1.0f / CALIBRATION_READINGS) * acc_y_cal);
  acc_z_cal = ((1.0f / CALIBRATION_READINGS) * acc_z_cal);
  if (abs(acc_x_cal) > abs(acc_y_cal) &&
      abs(acc_x_cal) > abs(acc_z_cal)) // normalize to x axis down
  {
    normalize(acc_x_cal);
    acc_y_cal *= -1;
    acc_z_cal *= -1;
  } else if (abs(acc_y_cal) > abs(acc_x_cal) &&
             abs(acc_y_cal) > abs(acc_z_cal)) {
    acc_x_cal *= -1;
    normalize(acc_y_cal);
    acc_z_cal *= -1;
  } else if (abs(acc_z_cal) > abs(acc_x_cal) &&
             abs(acc_z_cal) > abs(acc_y_cal)) {
    acc_x_cal *= -1;
    acc_y_cal *= -1;
    normalize(acc_z_cal);
  } else {
    // handle calibration failed path
  }

  printf("GYR | X:%8.2f  Y:%8.2f  Z:%8.2f    "
         "ACC | X:%7.3f  Y:%7.3f  Z:%7.3f\r\n",
         gyr_x_cal, gyr_y_cal, gyr_z_cal, acc_x_cal, acc_y_cal, acc_z_cal);
}
void vFilterGyro(void *pvParameters) {
  filter *imuFilter = ((filter *)pvParameters);
  uint8_t acc_gyro_dat[12];
  int64_t curr_time;
  float delta_time;
  int64_t last_time = esp_timer_get_time();

  for (;;) {
    uint8_t d[1];
    spi_read(0x1B, d, 1, bmi160_t);
    if (d[0] == 208) {
      spi_read(REG_GYR_ACC_DATA, acc_gyro_dat, 12, bmi160_t);
      curr_time = esp_timer_get_time();
      delta_time = curr_time - last_time;
      last_time = curr_time;
      int16_t raw_gx =
          ((((int16_t)((acc_gyro_dat[1] << 8) | acc_gyro_dat[0]) + gyr_x_cal)) /
           GYR_500_LSB) *
          DEG_TO_RAD;
      int16_t raw_gy =
          ((((int16_t)((acc_gyro_dat[3] << 8) | acc_gyro_dat[2]) + gyr_y_cal)) /
           GYR_500_LSB) *
          DEG_TO_RAD;
      int16_t raw_gz =
          ((((int16_t)((acc_gyro_dat[5] << 8) | acc_gyro_dat[4]) + gyr_z_cal)) /
           GYR_500_LSB) *
          DEG_TO_RAD;
      float acc_x =
          (((int16_t)((acc_gyro_dat[7] << 8) | acc_gyro_dat[6]) + acc_x_cal)) /
          ACC_2G_LSB_G;
      float acc_y =
          (((int16_t)((acc_gyro_dat[9] << 8) | acc_gyro_dat[8]) + acc_y_cal)) /
          ACC_2G_LSB_G;
      float acc_z = (((int16_t)((acc_gyro_dat[11] << 8) | acc_gyro_dat[10]) +
                      acc_z_cal)) /
                    ACC_2G_LSB_G;
      imuFilter->update_imu(raw_gx, raw_gy, raw_gz, acc_x, acc_y, acc_z,
                            delta_time / (1000.0 * 1000));
    } else
      vTaskDelay(1);
  }
}

extern "C" void app_main(void) {
  spi_initalize();
  setupBmi160();
  calibrate_gyro(gyr_x_cal, gyr_y_cal, gyr_z_cal, acc_x_cal, acc_y_cal,
                 acc_z_cal);
  // make a filter in this loop so we can use its angles
  filter imuFilter;
  xTaskCreate(vFilterGyro, "Filter task", 1048, &imuFilter, 1, nullptr);

  // printf("Quat: [w=% .5f, x=% .5f, y=% .5f, z=% .5f]\n", imuFilter.q0,
  // imuFilter.q1, imuFilter.q2, imuFilter.q3);
  while (1) {
    imuFilter.computeangles();
    printf("Roll: %7.2f deg | Pitch: %7.2f deg | Yaw: %7.2f deg\n",
           imuFilter.roll, imuFilter.pitch, imuFilter.yaw);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
