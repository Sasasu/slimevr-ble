#include <zephyr/device.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

#include <zephyr/logging/log.h>

struct icm20948_data {};

struct icm20948_config {
  const struct spi_dt_spec spi;
};
