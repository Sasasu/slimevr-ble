#include <zephyr/bluetooth/bluetooth.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/sys/printk.h>

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

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

  const struct spi_dt_spec icm20948_dev = SPI_DT_SPEC_GET(
      DT_NODELABEL(icm20948),
      SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
      // | SPI_FRAME_FORMAT_MOTOROLA | SPI_MODE_CPOL | SPI_MODE_CPHA,
      0);

#define READ (1 << 7)
#define WRITE (0 << 7)
#define REG_WHO_AM_I (0x00)
#define REG_BANK_SEL (0x7F)

#define VAL_BANK_0 (0x00 << 4)
#define VAL_BANK_1 (0x01 << 4)
#define VAL_BANK_2 (0x02 << 4)

  struct spi_buf_set bank0 = {
      .count = 1,
      .buffers = (struct spi_buf[1]){{
          .buf = (uint8_t[]){WRITE | REG_BANK_SEL, VAL_BANK_0},
          .len = 2,
      }},
  };

  struct spi_buf_set tx = {
      .count = 1,
      .buffers = (struct spi_buf[1]){{
          .buf = (uint8_t[]){READ | REG_WHO_AM_I},
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

  int i = 1;
  spi_write_dt(&icm20948_dev, &bank0);

  while (i++) {
    int e1 = spi_transceive_dt(&icm20948_dev, &tx, &rx);
    int r = ((uint8_t *)rx.buffers->buf)[1];
    printk("%d %d %x\n", i, e1, r);

    k_sleep(Z_TIMEOUT_MS(100));
  }
  // 0xEA

  return 0;
}
