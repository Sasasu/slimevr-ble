#include "zephyr/drivers/sensor.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/time_units.h"
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT slimevr_blesink

LOG_MODULE_REGISTER(blesink);

struct blesink_dev_data {
  const struct blesink_dev_cfg *cfg;

  uint8_t battery_level;

  struct k_work work;
  struct k_timer timer;
};

struct blesink_dev_cfg {
  const struct device *battery;
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),

    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  // Battery Service
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
                  // Physical Activity Monitor Service
                  BT_UUID_16_ENCODE(BT_UUID_PAMS_VAL)),
};

static int blesink_update_battery_level(const struct device *dev) {
  const struct blesink_dev_cfg *config = dev->config;
  struct blesink_dev_data *data = dev->data;
  struct sensor_value temp;

  int err = sensor_sample_fetch_chan(config->battery,
                                     SENSOR_CHAN_GAUGE_STATE_OF_CHARGE);
  if (err < 0)
    return err;

  err = sensor_channel_get(config->battery, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE,
                           &temp);
  if (err < 0)
    return err;

  data->battery_level = temp.val1;

  return bt_bas_set_battery_level(data->battery_level);
}

static void bt_connected(struct bt_conn *conn, uint8_t err) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (err) {
    LOG_WRN("Connection failed with %s (err 0x%02x)", addr, err);
  } else {
    LOG_INF("Connected with %s", addr);
  }
}

static void bt_disconnected(struct bt_conn *conn, uint8_t reason) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  LOG_INF("Disconnected from %s (reason 0x%02x)", addr, reason);
}

static struct bt_conn_cb conn_cb = {
    .connected = bt_connected,
    .disconnected = bt_disconnected,
};

static void bt_pairing_cancel(struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  LOG_INF("bt_pairing_cancel %s", addr);
};

static void bt_pairing_confirm(struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  LOG_INF("bt_pairing_confirm %s", addr);
};

static struct bt_conn_auth_cb auth_cb = {
    .cancel = bt_pairing_cancel,
    .pairing_confirm = bt_pairing_confirm,
};

static void bt_ready(int err) {
  if (err) {
    LOG_WRN("Bluetooth init failed (err %d)", err);
    return;
  }

  LOG_INF("Bluetooth initialized");

  err = bt_conn_auth_cb_register(&auth_cb);
  if (err) {
    LOG_WRN("bt_conn_auth_cb_register (err %d)", err);
    return;
  }

  bt_conn_cb_register(&conn_cb);
  if (err) {
    LOG_WRN("bt_conn_cb (err %d)", err);
    return;
  }

  err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    LOG_WRN("Advertising failed to start (err %d)", err);
    return;
  }

  LOG_INF("Advertising successfully started");
}

static int blesink_init(const struct device *dev) {
  const struct blesink_dev_cfg *config = dev->config;

  int err = bt_enable(bt_ready);
  if (err) {
    LOG_WRN("Bluetooth init failed (err %d)", err);
    return 0;
  }

  if (!device_is_ready(config->battery)) {
    LOG_WRN("battery device no ready");
    return -ENODEV;
  }

  // TODO move to main thread. use device API
  while (false) {
    k_sleep(K_SECONDS(1));
    blesink_update_battery_level(dev);
  }

  return blesink_update_battery_level(dev);
}

#define CREATE_BLESINK_DEVICE(inst)                                            \
  static const struct blesink_dev_cfg blesink_cfg_##inst = {                   \
      .battery = DEVICE_DT_GET(DT_INST_PROP(inst, battery)),                   \
  };                                                                           \
  static struct blesink_dev_data blesink_data_##inst = {                       \
      .cfg = &blesink_cfg_##inst,                                              \
  };                                                                           \
                                                                               \
  DEVICE_DT_INST_DEFINE(inst, blesink_init, NULL, &blesink_data_##inst,        \
                        &blesink_cfg_##inst, APPLICATION, 90, NULL);

DT_INST_FOREACH_STATUS_OKAY(CREATE_BLESINK_DEVICE)
