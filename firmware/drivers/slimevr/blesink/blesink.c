#include <zephyr/device.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT slimevr_blesink

LOG_MODULE_REGISTER(blesink);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),

    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  // Battery Service
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
                  // Physical Activity Monitor Service
                  BT_UUID_16_ENCODE(BT_UUID_PAMS_VAL)),
};

static void bt_connected(struct bt_conn *conn, uint8_t err) {
  if (err) {
    LOG_WRN("Connection failed (err 0x%02x)", err);
  } else {
    LOG_INF("Connected");
  }
}

static void bt_disconnected(struct bt_conn *conn, uint8_t reason) {
  LOG_INF("Disconnected (reason 0x%02x)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = bt_connected,
    .disconnected = bt_disconnected,
};

static void bt_ready(int err) {
  if (err) {
    LOG_WRN("Bluetooth init failed (err %d)", err);
    return;
  }

  LOG_INF("Bluetooth initialized");

  err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    LOG_WRN("Advertising failed to start (err %d)", err);
    return;
  }

  LOG_INF("Advertising successfully started");
}

static int blesink_init(const struct device *dev) {
  ARG_UNUSED(dev);

  int err = bt_enable(bt_ready);
  if (err) {
    LOG_WRN("Bluetooth init failed (err %d)", err);
    return 0;
  }

  return 0;
}

struct blesink_dev_data {};

struct blesink_dev_cfg {};

#define CREATE_BLESINK_DEVICE(inst)                                            \
  static struct blesink_dev_data blesink_data_##inst = {};                     \
  static const struct blesink_dev_cfg blesink_cfg_##inst = {};                 \
  DEVICE_DT_INST_DEFINE(inst, blesink_init, NULL, &blesink_data_##inst,        \
                        &blesink_cfg_##inst, APPLICATION, 90, NULL);           \

DT_INST_FOREACH_STATUS_OKAY(CREATE_BLESINK_DEVICE)