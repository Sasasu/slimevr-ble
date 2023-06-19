#include "./icm20948.h"
#include "icm20948_reg.h"
#include <errno.h>
#include <stdint.h>

#define DT_DRV_COMPAT slimevr_icm20948

LOG_MODULE_DECLARE(ICM20948, CONFIG_ICM20948_LOG_LEVEL);

static int icm20948_spi_read_byte(const struct device *dev, uint8_t reg,
                                  uint8_t *out) {
  const struct icm20948_config *config = dev->config;

  struct spi_buf_set tx = {
      .count = 1,
      .buffers = (struct spi_buf[1]){{
          .buf = (uint8_t[]){reg | READ},
          .len = 1,
      }},
  };

  struct spi_buf_set rx = {
      .count = 1,
      .buffers = (struct spi_buf[1]){{
          .buf = (uint8_t[2]){0},
          .len = 2,
      }},
  };

  int e = spi_transceive_dt(&config->spi, &tx, &rx);

  if (!e) {
    return e;
  }

  if (out) {
    *out = ((uint8_t *)rx.buffers->buf)[1];
  }

  return 0;
}

static int icm20948_spi_write_byte(const struct device *dev, uint8_t reg,
                                   uint8_t value) {
  const struct icm20948_config *config = dev->config;
  struct spi_buf_set tx = {
      .count = 1,
      .buffers = (struct spi_buf[1]){{
          .buf = (uint8_t[]){reg | WRITE, value},
          .len = 2,
      }},
  };

  return spi_write_dt(&config->spi, &tx);
}

int icm20948_init(const struct device *dev) {
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
    LOG_ERR("not find ICM20948");
    return -ENODEV;
  }

  // disable I2C interface
  ICM20948_USER_CTRL_t x = {.I2C_IF_DIS = 1};
  icm20948_spi_write_byte(dev, USER_CTRL_B0, IN(x));

  return 0;
}

static const struct sensor_driver_api icm20948_driver_api = {};

/* device defaults to spi mode 0/3 support */
#define ICM20948_SPI_CFG                                                       \
  SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) |       \
      SPI_TRANSFER_MSB

#define INIT_ICM20948_INST(inst)                                               \
  static struct icm20948_data icm20948_data_##inst;                            \
  static const struct icm20948_config icm20948_cfg_##inst = {                  \
      .spi = SPI_DT_SPEC_INST_GET(inst, ICM20948_SPI_CFG, 0),                  \
  };                                                                           \
                                                                               \
  SENSOR_DEVICE_DT_INST_DEFINE(inst, icm20948_init, NULL,                      \
                               &icm20948_data_##inst, &icm20948_cfg_##inst,    \
                               POST_KERNEL, 90, &icm20948_driver_api);         \
                                                                               \
  LOG_INSTANCE_REGISTER(ICM20948, icm20948_##int, CONFIG_ICM20948_LOG_LEVEL);

DT_INST_FOREACH_STATUS_OKAY(INIT_ICM20948_INST)
