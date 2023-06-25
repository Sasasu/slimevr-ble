#include "./icm20948.h"
#include "icm20948_reg.h"
#include "zephyr/logging/log.h"
#include <stdint.h>

#define DT_DRV_COMPAT slimevr_icm20948

LOG_MODULE_REGISTER(icm20948);

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

  if (e) {
    LOG_ERR("spi_transceive_dt %d", e);
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

static int icm20948_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val) {

  switch ((enum sensor_channel_icm20948)chan) {
  case (enum sensor_channel_icm20948)SENSOR_CHAN_DIE_TEMP:
  case SENSOR_CHAN_GAME_ROTATION_QUAT_X:
  case SENSOR_CHAN_GAME_ROTATION_QUAT_Y:
  case SENSOR_CHAN_GAME_ROTATION_QUAT_Z:
  case SENSOR_CHAN_GAME_ROTATION_QUAT_W:
    break;
  default:
    return -ENOTSUP;
  }

  return 0;
}

static void icm20948_gpio_callback(const struct device *dev,
                                   struct gpio_callback *cb, uint32_t pins) {
  struct icm20948_data *data = CONTAINER_OF(cb, struct icm20948_data, gpio_cb);

  // icm20948_spi_read_byte FIFO

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

  gpio_pin_interrupt_configure_dt(&config->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);

  return 0;
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

  // disable I2C interface
  ICM20948_USER_CTRL_t x = {.I2C_IF_DIS = 1};
  icm20948_spi_write_byte(dev, USER_CTRL_B0, IN(x));

  // TODO load FW, enable FIFO

  return icm20948_trigger_init(dev);
}

static const struct sensor_driver_api icm20948_driver_api = {
    .channel_get = icm20948_channel_get,
    .trigger_set = icm20948_trigger_set,
};

#define ICM20948_SPI_CFG SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)

#define INIT_ICM20948_INST(inst)                                               \
  static struct icm20948_data icm20948_data_##inst;                            \
  static const struct icm20948_config icm20948_cfg_##inst = {                  \
      .spi = SPI_DT_SPEC_INST_GET(inst, ICM20948_SPI_CFG, 0),                  \
      .gpio_int = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),              \
  };                                                                           \
                                                                               \
  SENSOR_DEVICE_DT_INST_DEFINE(inst, icm20948_init, NULL,                      \
                               &icm20948_data_##inst, &icm20948_cfg_##inst,    \
                               APPLICATION, 90, &icm20948_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INIT_ICM20948_INST)
