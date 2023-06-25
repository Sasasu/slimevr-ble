#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

struct icm20948_data {
  sensor_trigger_handler_t data_ready_handler;
  const struct sensor_trigger *data_ready_trigger;

  struct gpio_callback gpio_cb;
  // TODO 4*32 bytes float data
};

struct icm20948_config {
  const struct spi_dt_spec spi;
  const struct gpio_dt_spec gpio_int;
};

enum sensor_channel_icm20948 {
  __SENSOR_CHAN_DIE_TEMP = SENSOR_CHAN_DIE_TEMP,
  SENSOR_CHAN_GAME_ROTATION_QUAT_X = SENSOR_CHAN_PRIV_START,
  SENSOR_CHAN_GAME_ROTATION_QUAT_Y,
  SENSOR_CHAN_GAME_ROTATION_QUAT_Z,
  SENSOR_CHAN_GAME_ROTATION_QUAT_W,
};
