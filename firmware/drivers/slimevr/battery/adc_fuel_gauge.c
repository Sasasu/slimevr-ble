#include "zephyr/devicetree/io-channels.h"
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>

#include <stdint.h>

#define DT_DRV_COMPAT slimevr_adc_fuel_gauge

LOG_MODULE_REGISTER(adc_fuel_gauge);

struct battery_level_point {
  /** Remaining life at #lvl_mV. */
  uint16_t lvl_pptt;

  /** Battery voltage at #lvl_pptt remaining life. */
  uint16_t lvl_mV;
};

unsigned int battery_level_pptt(unsigned int batt_mV,
                                const struct battery_level_point *curve) {
  const struct battery_level_point *pb = curve;

  if (batt_mV >= pb->lvl_mV) {
    return pb->lvl_pptt;
  }

  /* Go down to the last point at or below the measured voltage. */
  while ((pb->lvl_pptt > 0) && (batt_mV < pb->lvl_mV)) {
    ++pb;
  }

  if (batt_mV < pb->lvl_mV) {
    /* Below lowest point, cap at minimum */
    return pb->lvl_pptt;
  }

  /* Linear interpolation between below and above points. */
  const struct battery_level_point *pa = pb - 1;

  return pb->lvl_pptt + ((pa->lvl_pptt - pb->lvl_pptt) *
                         (batt_mV - pb->lvl_mV) / (pa->lvl_mV - pb->lvl_mV));
}

struct adc_fuel_gauge_config {
  const struct adc_dt_spec io_channel;
  uint32_t output_ohms;
  uint32_t full_ohms;

  uint32_t battery_full_mv;
  uint32_t battery_empty_mv;
};

struct adc_fuel_gauge_data {
  int32_t val;
};

int adc_fuel_gauge_fetch(const struct device *dev, enum sensor_channel chan) {
  const struct adc_fuel_gauge_config *config = dev->config;
  struct adc_fuel_gauge_data *data = dev->data;

  if (chan != SENSOR_CHAN_GAUGE_STATE_OF_CHARGE && chan != SENSOR_CHAN_ALL) {
    return -ENODATA;
  }

  struct adc_sequence sequence = {
      .buffer = (uint16_t[1]){0},
      .buffer_size = sizeof(uint16_t),
  };

  adc_sequence_init_dt(&config->io_channel, &sequence);
  int err = adc_read(config->io_channel.dev, &sequence);
  if (err < 0) {
    LOG_WRN("ADC read %d", err);
    return err;
  }

  int32_t val_mv;

  if (config->io_channel.channel_cfg.differential) {
    val_mv = (int32_t)(((int16_t *)sequence.buffer)[0]);
  } else {
    val_mv = (int32_t)(((uint16_t *)sequence.buffer)[0]);
  }

  err = adc_raw_to_millivolts_dt(&config->io_channel, &val_mv);
  if (err < 0) {
    LOG_WRN("ADC to mv %d", err);
    return err;
  }

  val_mv = (val_mv * config->full_ohms) / config->output_ohms;

  data->val = val_mv;

  return 0;
}

int adc_fuel_gauge_get(const struct device *dev, enum sensor_channel chan,
                       struct sensor_value *val) {
  const struct adc_fuel_gauge_config *config = dev->config;
  struct adc_fuel_gauge_data *data = dev->data;

  if (chan != SENSOR_CHAN_GAUGE_STATE_OF_CHARGE) {
    return -ENODATA;
  }

  const struct battery_level_point levels[] = {
      {100 * 512, config->battery_full_mv},
      {50 * 512, 3950},
      {31 * 512, 3550},
      {0, config->battery_empty_mv},
  };

  val->val1 = battery_level_pptt(data->val, levels) / 512;
  val->val2 = 0;

  return 0;
}

static int adc_fuel_gauge_init(const struct device *dev) {
  const struct adc_fuel_gauge_config *config = dev->config;

  LOG_INF("adc based fuel gauge start initialize");

  if (!device_is_ready(config->io_channel.dev)) {
    LOG_WRN("ADC device not ready");
    return -ENODEV;
  }

  int err = adc_channel_setup_dt(&config->io_channel);
  if (err < 0) {
    LOG_WRN("ADC device setup %d", err);
    return err;
  }

  err = adc_fuel_gauge_fetch(dev, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE);
  if (err < 0) {
    LOG_WRN("ADC read %d", err);
    return err;
  }

  struct sensor_value val;
  err = adc_fuel_gauge_get(dev, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &val);
  LOG_INF("ADC device initialized. current battery level %d%%", val.val1);

  return err;
}

static const struct sensor_driver_api adc_fuel_gauge_api = {
    .channel_get = adc_fuel_gauge_get,
    .sample_fetch = adc_fuel_gauge_fetch,
};

#define INIT_ADC_FUEL_GAUGE_INST(inst)                                         \
  static struct adc_fuel_gauge_data adc_fuel_gauge_data_##inst = {};           \
  static const struct adc_fuel_gauge_config adc_fuel_gauge_config_##inst = {   \
      .io_channel = ADC_DT_SPEC_INST_GET(inst),                                \
      .output_ohms = DT_INST_PROP(inst, output_ohms),                          \
      .full_ohms = DT_INST_PROP(inst, full_ohms),                              \
      .battery_full_mv = DT_INST_PROP(inst, battery_full_mv),                  \
      .battery_empty_mv = DT_INST_PROP(inst, battery_empty_mv),                \
  };                                                                           \
                                                                               \
  SENSOR_DEVICE_DT_INST_DEFINE(                                                \
      inst, adc_fuel_gauge_init, NULL, &adc_fuel_gauge_data_##inst,            \
      &adc_fuel_gauge_config_##inst, APPLICATION, 80, &adc_fuel_gauge_api);

DT_INST_FOREACH_STATUS_OKAY(INIT_ADC_FUEL_GAUGE_INST)
