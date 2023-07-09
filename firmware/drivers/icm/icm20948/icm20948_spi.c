#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

#include <zephyr/logging/log.h>

#include "./icm20948.h"
#include "./icm20948_dmp.h"
#include "./icm20948_reg.h"

#define DT_DRV_COMPAT slimevr_icm20948

LOG_MODULE_REGISTER(icm20948);

ICM_20948_REG_BANK_SEL_t b0 = {.USER_BANK = 0};
ICM_20948_REG_BANK_SEL_t b1 = {.USER_BANK = 1};
ICM_20948_REG_BANK_SEL_t b2 = {.USER_BANK = 2};
ICM_20948_REG_BANK_SEL_t b3 = {.USER_BANK = 3};

static const uint8_t dmp3_image[] = {
#include "./icm20948_img.dmp3a.h"
};

static int icm20948_spi_read_byte_burst(const struct device *dev, uint8_t reg,
                                        int n, uint8_t *out) {
  const struct icm20948_config *config = dev->config;

  struct spi_buf_set tx = {
      .count = 1,
      .buffers = (struct spi_buf[1]){{
          .buf = (uint8_t[]){reg | READ},
          .len = 1,
      }},
  };

  struct spi_buf_set rx = {
      .count = 2,
      .buffers =
          (struct spi_buf[2]){
              {
                  .buf = (uint8_t[1]){0},
                  .len = 1,
              },
              {
                  .buf = out,
                  .len = n,
              },
          },
  };

  return spi_transceive_dt(&config->spi, &tx, &rx);
}

static int icm20948_spi_write_byte_burst(const struct device *dev, uint8_t reg,
                                         int n, const uint8_t *const in) {
  const struct icm20948_config *config = dev->config;
  struct spi_buf_set tx = {
      .count = 2,
      .buffers =
          (struct spi_buf[2]){
              {
                  .buf = (uint8_t[]){reg | WRITE},
                  .len = 1,
              },
              {
                  .buf = (void *)in,
                  .len = n,
              },
          },
  };

  return spi_write_dt(&config->spi, &tx);
}

static int icm20948_spi_read_byte(const struct device *dev, uint8_t reg,
                                  uint8_t *out) {
  return icm20948_spi_read_byte_burst(dev, reg, 1, out);
}

static int icm20948_spi_write_byte(const struct device *dev, uint8_t reg,
                                   uint8_t value) {
  return icm20948_spi_write_byte_burst(dev, reg, 1, &value);
}

static int icm20948_spi_mem_read_byte_burst(const struct device *dev,
                                            uint16_t reg, int n, uint8_t *out) {
  icm20948_spi_write_byte(dev, REG_BANK_SEL_B0, IN(b0));

  int membank = (reg >> 8);
  icm20948_spi_write_byte(dev, REG_MEM_BANK_SEL_B0, IN(membank));

  int bytes_readed = 0;
  while (bytes_readed < n) {
    int memoffset = reg & 0xFF;
    icm20948_spi_write_byte(dev, REG_MEM_START_ADDR_B0, memoffset);

    int remaining = n - bytes_readed;
    int len = (remaining > MAX_MEM_WRITE ? MAX_MEM_WRITE : remaining);
    icm20948_spi_read_byte_burst(dev, REG_MEM_R_W_B0, len, &out[bytes_readed]);

    bytes_readed += len;
    reg += len;
  }

  return 0;
}

static int icm20948_spi_mem_write_byte_burst(const struct device *dev,
                                             uint16_t reg, int n,
                                             const uint8_t *const in) {
  icm20948_spi_write_byte(dev, REG_BANK_SEL_B0, IN(b0));

  int membank = (reg >> 8);
  icm20948_spi_write_byte(dev, REG_MEM_BANK_SEL_B0, IN(membank));

  int bytes_written = 0;
  while (bytes_written < n) {
    int memoffset = reg & 0xFF;
    icm20948_spi_write_byte(dev, REG_MEM_START_ADDR_B0, memoffset);

    int remaining = n - bytes_written;
    int len = (remaining > MAX_MEM_WRITE ? MAX_MEM_WRITE : remaining);
    icm20948_spi_write_byte_burst(dev, REG_MEM_R_W_B0, len, &in[bytes_written]);

    bytes_written += len;
    reg += len;
  }

  return 0;
}

static int icm20948_chip_reset(const struct device *dev) {
  // Switch to bank 0
  icm20948_spi_write_byte(dev, REG_BANK_SEL_B0, IN(b0));

  // fully reset
  ICM20948_PWR_MGMT_1_t reset = {
      .DEVICE_RESET = 1,
  };
  icm20948_spi_write_byte(dev, PWR_MGMT_1_B0, IN(reset));

  return 0;
}

static int icm20948_initializing(const struct device *dev) {
  icm20948_chip_reset(dev);

  // Switch to bank 0
  icm20948_spi_write_byte(dev, REG_BANK_SEL_B0, IN(b0));

  // Disable I2C interface
  ICM20948_USER_CTRL_t x = {.I2C_IF_DIS = 1};
  icm20948_spi_write_byte(dev, USER_CTRL_B0, IN(x));

  // Configure clock source through PWR_MGMT_1.
  ICM20948_PWR_MGMT_1_t clk = {
      .CLKSEL = ICM20948_MGMT_1_B0_CLKSET_Auto,
  };
  icm20948_spi_write_byte(dev, PWR_MGMT_1_B0, IN(clk));

  // Enable accel and gyro sensors through PWR_MGMT_2.
  ICM20948_PWR_MGMT_2_t sen = {
      .DIABLE_ACCEL = 0,
      .DISABLE_GYRO = 0,
  };
  icm20948_spi_write_byte(dev, PWR_MGMT_2_B0, IN(sen));

  // Configure Gyro/Accel in Low power mode (duty cycled mode) with LP_CONFIG.
  ICM20948_LP_CONFIG_t lp = {
      .I2C_MST_CYCLE = 1,
      .ACCEL_CYCLE = 1,
      .GYRO_CYCLE = 1,
  };
  icm20948_spi_write_byte(dev, LP_CONFIG_B0, IN(lp));

  // Switch to b2
  icm20948_spi_write_byte(dev, REG_BANK_SEL_B0, IN(b2));

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1.
  ICM20948_GYRO_CONFIG_1_t gyro = {
      .GYRO_FS_SEL = 0b11, // 2000dps
  };
  icm20948_spi_write_byte(dev, GYRO_CONFIG_1_B2, IN(gyro));

  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG.
  ICM20948_ACCEL_CONFIG_t acc = {
      .ACCEL_FS_SEL = 0b01, // 4g
  };
  icm20948_spi_write_byte(dev, ACCEL_CONFIG_B2, IN(acc));

  return 0;
}

static int icm20948_enable_fifo(const struct device *dev) {
  // Switch to b0
  icm20948_spi_write_byte(dev, REG_BANK_SEL_B0, IN(b0));

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2.
  ICM20948_INT_ENABLE_2_t int1 = {.FIFO_OVERFLOW_EN = 1};
  icm20948_spi_write_byte(dev, INT_ENABLE_1_B0, IN(int1));

  // Turn off what goes into the FIFO through FIFO_EN, FIFO_EN_2.
  ICM20948_FIFO_EN_1_t en1 = {
      .SLV_0_FIFO_EN = 0,
      .SLV_1_FIFO_EN = 0,
      .SLV_2_FIFO_EN = 0,
      .SLV_3_FIFO_EN = 0,
  };
  icm20948_spi_write_byte(dev, INT_ENABLE_1_B0, IN(en1));
  ICM_20948_FIFO_EN_2_t en2 = {
      .ACCEL_FIFO_EN = 0,
      .GYRO_X_FIFO_EN = 0,
      .GYRO_Y_FIFO_EN = 0,
      .GYRO_Z_FIFO_EN = 0,
      .TEMP_FIFO_EN = 0,
  };
  icm20948_spi_write_byte(dev, INT_ENABLE_2_B0, IN(en2));

  // Turn off data ready interrupt through INT_ENABLE_1.
  ICM20948_INT_ENABLE_1_t int2 = {.RAW_DATA_0_RDY_EN = 0};
  icm20948_spi_write_byte(dev, INT_ENABLE_1_B0, IN(int2));

  // Turn on interrupt for DMP
  ICM20948_INT_ENABLE_t int3 = {.DMP_INT1_EN = 1};
  icm20948_spi_write_byte(dev, INT_ENABLE_B0, IN(int3));

  // Reset FIFO through FIFO_RST.
  // examples write 0x1F followed by 0x1E
  ICM20948_FIFO_RST_t rst = {.FIFO_RESET = 0b11111};
  icm20948_spi_write_byte(dev, FIFO_RST_B0, IN(rst));
  rst = (ICM20948_FIFO_RST_t){.FIFO_RESET = 0b11110};
  icm20948_spi_write_byte(dev, FIFO_RST_B0, IN(rst));

  return 0;
}

static int icm20948_config_sample_rate(const struct device *dev) {
  icm20948_spi_write_byte(dev, REG_BANK_SEL_B0, IN(b2));

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV.
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2.
  // gyrodiv = 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
  // accdiv = 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
  ICM20948_ACCEL_SMPLRT_DIV_2_t accdiv = 4; // 225Hz
  icm20948_spi_write_byte(dev, ACCEL_SMPLRT_DIV_2_B2, IN(accdiv));
  ICM20948_GYRO_SMPLRT_DIV_t gyrodiv = 4; // 225Hz
  icm20948_spi_write_byte(dev, GYRO_SMPLRT_DIV_B2, IN(gyrodiv));

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL.
  unsigned char start_address[2];
  start_address[0] = (unsigned char)(DMP_START_ADDRESS >> 8);
  start_address[1] = (unsigned char)(DMP_START_ADDRESS & 0xff);
  icm20948_spi_write_byte_burst(dev, REG_PRGM_START_ADDRH_B2, 2, start_address);

  return 0;
}

static int icm20948_load_dmp_fw(const struct device *dev) {
  icm20948_spi_write_byte(dev, REG_BANK_SEL_B0, IN(b0));

  for (int offset = 0; offset < sizeof(dmp3_image); ++offset) {
    uint16_t memoffset = offset + DMP_LOAD_START;
    uint8_t b = dmp3_image[offset];

    uint8_t bank = memoffset >> 8;
    uint8_t inbankaddr = memoffset & 0xFF;
    icm20948_spi_write_byte(dev, REG_MEM_BANK_SEL_B0, bank);
    icm20948_spi_write_byte(dev, REG_MEM_START_ADDR_B0, inbankaddr);
    icm20948_spi_write_byte(dev, REG_MEM_R_W_B0, b);
  }

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL. (again?)
  unsigned char start_address[2];
  start_address[0] = (unsigned char)(DMP_START_ADDRESS >> 8);
  start_address[1] = (unsigned char)(DMP_START_ADDRESS & 0xff);
  icm20948_spi_write_byte_burst(dev, REG_PRGM_START_ADDRH_B2, 2, start_address);

  return 0;
}

static int icm20948_set_fixdata(const struct device *dev) {
  // Configure Accel scaling to DMP
  const uint8_t acc_scale[4] = {0x04, 0x00, 0x00, 0x00};  // 0x4000000
  const uint8_t acc_scale2[4] = {0x00, 0x04, 0x00, 0x00}; // 0x40000
  icm20948_spi_mem_write_byte_burst(dev, ACC_SCALE, 4, acc_scale);
  icm20948_spi_mem_write_byte_burst(dev, ACC_SCALE2, 4, acc_scale2);

  // Configure Compass mount matrix and scale to DMP
  const uint8_t mount_mul_zero[4] = {0x00, 0x00, 0x00, 0x00};
  const uint8_t mount_mul_plus[4] = {0x09, 0x99, 0x99, 0x99};
  const uint8_t mount_mul_minus[4] = {0xF6, 0x66, 0x66, 0x67};

  icm20948_spi_mem_write_byte_burst(dev, CPASS_MTX_00, 4, mount_mul_plus);
  icm20948_spi_mem_write_byte_burst(dev, CPASS_MTX_01, 4, mount_mul_zero);
  icm20948_spi_mem_write_byte_burst(dev, CPASS_MTX_02, 4, mount_mul_zero);
  icm20948_spi_mem_write_byte_burst(dev, CPASS_MTX_10, 4, mount_mul_zero);
  icm20948_spi_mem_write_byte_burst(dev, CPASS_MTX_11, 4, mount_mul_minus);
  icm20948_spi_mem_write_byte_burst(dev, CPASS_MTX_12, 4, mount_mul_zero);
  icm20948_spi_mem_write_byte_burst(dev, CPASS_MTX_20, 4, mount_mul_zero);
  icm20948_spi_mem_write_byte_burst(dev, CPASS_MTX_21, 4, mount_mul_zero);
  icm20948_spi_mem_write_byte_burst(dev, CPASS_MTX_22, 4, mount_mul_minus);

  return 0;
}

static int icm20948_set_output(const struct device *dev) {
  // Configuring DMP to output data to FIFO
  // 32-bit 9-axis quaternion + heading accuracy 0x0400
  const uint8_t axis9_quaternion[2] = {0x40, 0x00};
  icm20948_spi_mem_write_byte_burst(dev, DATA_OUT_CTL1, 2, axis9_quaternion);

  const uint8_t data_ready[2] = {0x0a, 0x00}; // Accel + Gyro
  icm20948_spi_mem_write_byte_burst(dev, DATA_RDY_STATUS, 2, data_ready);

  return 0;
}

static int icm20948_get_fifo_count(const struct device *dev, int *cnt) {
  ICM20948_FIFO_COUNT_H_t ch = {.FIFO_CNT = 0};
  ICM20948_FIFO_COUNT_L_t cl = 0;

  icm20948_spi_read_byte(dev, FIFO_COUNTL_B0, OUT(cl));
  icm20948_spi_read_byte(dev, FIFO_COUNTH_B0, OUT(ch));

  *cnt = ((int)ch.FIFO_CNT << 8) | cl;

  return 0;
}

static int icm20948_fifo_read(const struct device *dev) {
  struct icm20948_data *data = dev->data;

  // 4bytes header + 14bytes data
  uint8_t buffer[4 + 14] = {};
  int avalible = 0;

  icm20948_get_fifo_count(dev, &avalible);
  if (avalible < sizeof(buffer)) {
    return -ENODATA;
  }

  icm20948_spi_write_byte(dev, REG_BANK_SEL_B0, IN(b0));
  icm20948_spi_mem_read_byte_burst(dev, FIFO_R_W_B0, sizeof(buffer), buffer);

  uint16_t *_b = (uint16_t *)buffer;
  int32_t x = _b[1] << 16 | _b[2];
  int32_t y = _b[3] << 16 | _b[4];
  int32_t z = _b[5] << 16 | _b[6];

  data->quaternion[0] = x;
  data->quaternion[1] = y;
  data->quaternion[2] = z;

  return 0;
}

static int icm20948_dmp_init(const struct device *dev) {
  icm20948_initializing(dev);
  icm20948_enable_fifo(dev);
  icm20948_config_sample_rate(dev);
  icm20948_load_dmp_fw(dev);
  icm20948_set_fixdata(dev);
  icm20948_set_output(dev);

  return 0;
}

static int icm20948_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val) {

#define DMPNUMBERTODOUBLECONVERTER 1073741824.0

  struct icm20948_data *data = dev->data;

  switch ((enum sensor_channel_icm20948)chan) {
  case SENSOR_CHAN_GAME_ROTATION_QUAT_X:
    val->val1 = data->quaternion[0];
    break;
  case SENSOR_CHAN_GAME_ROTATION_QUAT_Y:
    val->val1 = data->quaternion[1];
    break;
  case SENSOR_CHAN_GAME_ROTATION_QUAT_Z:
    val->val1 = data->quaternion[2];
    break;
  case SENSOR_CHAN_GAME_ROTATION_QUAT_W:
    val->val1 = 0; // TODO sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
    break;
  default:
    return -ENOTSUP;
  }

  return 0;
}

static void icm20948_gpio_callback(const struct device *dev,
                                   struct gpio_callback *cb, uint32_t pins) {
  struct icm20948_data *data = CONTAINER_OF(cb, struct icm20948_data, gpio_cb);

  icm20948_fifo_read(dev);

  data->data_ready_handler(dev, data->data_ready_trigger);
}

static int icm20948_trigger_set(const struct device *dev,
                                const struct sensor_trigger *trig,
                                sensor_trigger_handler_t handler) {
  struct icm20948_data *data = dev->data;
  const struct icm20948_config *config = dev->config;

  if (trig == NULL || handler == NULL) {
    return -EINVAL;
  }

  gpio_pin_interrupt_configure_dt(&config->gpio_int, GPIO_INT_DISABLE);

  switch (trig->type) {
  case SENSOR_TRIG_DATA_READY:
    data->data_ready_handler = handler;
    data->data_ready_trigger = trig;
    break;
  default:
    return -ENOTSUP;
  }

  return gpio_pin_interrupt_configure_dt(&config->gpio_int,
                                         GPIO_INT_EDGE_TO_ACTIVE);
}

static int icm20948_trigger_init(const struct device *dev) {
  struct icm20948_data *data = dev->data;
  const struct icm20948_config *config = dev->config;

  if (!config->gpio_int.port) {
    LOG_ERR("trigger enabled but no interrupt gpio supplied");
    return -ENODEV;
  }

  if (!gpio_is_ready_dt(&config->gpio_int)) {
    LOG_ERR("trigger enabled but gpio_int not ready");
    return -ENODEV;
  }

  gpio_pin_configure_dt(&config->gpio_int, GPIO_INPUT);
  gpio_init_callback(&data->gpio_cb, icm20948_gpio_callback,
                     BIT(config->gpio_int.pin));

  int r = gpio_add_callback(config->gpio_int.port, &data->gpio_cb);
  if (r < 0) {
    LOG_ERR("Failed to set gpio callback");
    return r;
  }

  return gpio_pin_interrupt_configure_dt(&config->gpio_int,
                                         GPIO_INT_EDGE_TO_ACTIVE);
}

static int icm20948_init(const struct device *dev) {
  const struct icm20948_config *config = dev->config;
  uint8_t out = 0;

  if (!spi_is_ready_dt(&config->spi)) {
    LOG_ERR("SPI bus is not ready");
    return -ENODEV;
  }

  ICM_20948_REG_BANK_SEL_t b = {.USER_BANK = 0};
  icm20948_spi_write_byte(dev, REG_BANK_SEL_B0, IN(b));
  icm20948_spi_read_byte(dev, WHO_AM_I_B0, &out);

  if (out != ICM20948_ID) {
    LOG_ERR("not find ICM20948 %x", out);
    return -ENODEV;
  }

  LOG_INF("find ICM20948, start initialize");

  icm20948_dmp_init(dev);

  return icm20948_trigger_init(dev);
}

static const struct sensor_driver_api icm20948_driver_api = {
    .channel_get = icm20948_channel_get,
    .trigger_set = icm20948_trigger_set,
};

#define ICM20948_SPI_CFG SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)

#define INIT_ICM20948_INST(inst)                                               \
  static struct icm20948_data icm20948_data_##inst = {};                       \
  static const struct icm20948_config icm20948_cfg_##inst = {                  \
      .spi = SPI_DT_SPEC_INST_GET(inst, ICM20948_SPI_CFG, 0),                  \
      .gpio_int = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),              \
  };                                                                           \
                                                                               \
  SENSOR_DEVICE_DT_INST_DEFINE(inst, icm20948_init, NULL,                      \
                               &icm20948_data_##inst, &icm20948_cfg_##inst,    \
                               APPLICATION, 80, &icm20948_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INIT_ICM20948_INST)
