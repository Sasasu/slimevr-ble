#include <zephyr/bluetooth/bluetooth.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/sys/printk.h>

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),

    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  // Battery Service
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
                  // Physical Activity Monitor Service
                  BT_UUID_16_ENCODE(BT_UUID_PAMS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err) {
  if (err) {
    printk("Connection failed (err 0x%02x)\n", err);
  } else {
    printk("Connected\n");
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
  printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(int err) {
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }

  printk("Bluetooth initialized\n");

  err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    printk("Advertising failed to start (err %d)\n", err);
    return;
  }

  printk("Advertising successfully started\n");
}

int main(void) {
  int err;

  printk("Starting SlimeVR Tracker\n");

  /* Initialize the Bluetooth Subsystem */
  err = bt_enable(bt_ready);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }

  return 0;
}
