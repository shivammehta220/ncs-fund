/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
/* STEP 3 - Include the header file of the I2C API */
#include <zephyr/drivers/i2c.h>
/* STEP 4.1 - Include the header file of printk() */
#include <zephyr/sys/printk.h>
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* STEP 8 - Define the I2C slave device address and the addresses of relevant registers */
/* AHT10 commands */
#define AHT10_CMD_SOFT_RESET     0xBA
#define AHT10_CMD_INIT           0xE1  /* Followed by 0x08, 0x00 */
#define AHT10_CMD_TRIGGER_MEAS   0xAC  /* Followed by 0x33, 0x00 */

/* Delays (minimum recommended) */
#define AHT10_RESET_DELAY_MS     20
#define AHT10_INIT_DELAY_MS      20
#define AHT10_MEAS_DELAY_MS      80
/* STEP 6 - Get the node identifier of the sensor */
#define I2C_NODE DT_NODELABEL(mysensor)

int main(void)
{

	int ret;

	/* STEP 7 - Retrieve the API-specific device structure and make sure that the device is
	 * ready to use  */
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return -1;
	}
	
	printk("AHT10 sample starting...\n");

    /* --- Soft Reset --- */
    uint8_t reset_cmd = AHT10_CMD_SOFT_RESET;
    ret = i2c_write_dt(&dev_i2c, &reset_cmd, 1);
    if (ret < 0) {
        printk("Error sending soft reset: %d\n", ret);
        return -1;
    }
    k_msleep(AHT10_RESET_DELAY_MS);

    /* --- Calibration/Initialization ---
     * Send: 0xE1, 0x08, 0x00
     */
    uint8_t init_cmd[3] = {AHT10_CMD_INIT, 0x08, 0x00};
    ret = i2c_write_dt(&dev_i2c, init_cmd, sizeof(init_cmd));
    if (ret < 0) {
        printk("Error sending init command: %d\n", ret);
        return -1;
    }
    k_msleep(AHT10_INIT_DELAY_MS);

    printk("AHT10 initialization complete.\n");

	while (1) {
		 /* --- Trigger Measurement ---
         * Send: 0xAC, 0x33, 0x00
         */
        uint8_t measure_cmd[3] = {AHT10_CMD_TRIGGER_MEAS, 0x33, 0x00};
        ret = i2c_write_dt(&dev_i2c, measure_cmd, sizeof(measure_cmd));
        if (ret < 0) {
            printk("Error triggering measurement: %d\n", ret);
            goto sleep_and_retry;
        }

        /* Wait for measurement to complete (~80 ms) */
        k_msleep(AHT10_MEAS_DELAY_MS);

        /* --- Read 6 bytes of data ---
         * Byte 0: status
         * Byte 1,2,3 (bits [19:0] of humidity)
         * Byte 3,4,5 (bits [19:0] of temperature)
         */
        uint8_t data[6] = {0};
        ret = i2c_read_dt(&dev_i2c, data, sizeof(data));
        if (ret < 0) {
            printk("Error reading data: %d\n", ret);
            goto sleep_and_retry;
        }

        /* STEP 11 - Parse humidity and temperature from the 6 bytes */
        /* 20-bit humidity = bits [19:0], located in data[1], data[2], data[3](upper 4 bits) */
        uint32_t raw_humidity = ((uint32_t)data[1] << 12) |
                                ((uint32_t)data[2] << 4)  |
                                ((uint32_t)(data[3] >> 4) & 0x0F);

        /* 20-bit temperature = bits [19:0], located in data[3](lower 4 bits), data[4], data[5] */
        uint32_t raw_temp = (((uint32_t)data[3] & 0x0F) << 16) |
                            ((uint32_t)data[4] << 8)     |
                            ((uint32_t)data[5]);

        /* Convert to %RH (humidity) and °C (temperature).
         * humidity = (raw_humidity / 2^20) * 100%
         * temperature = ((raw_temp / 2^20) * 200) - 50
         */
        float humidity = ((float)raw_humidity * 100.0f) / 1048576.0f; /* 2^20 = 1048576 */
        float temperature = (((float)raw_temp * 200.0f) / 1048576.0f) - 50.0f;

        printk("Humidity: %.2f %%RH, Temperature: %.2f °C\n", (double)humidity, (double)temperature);

sleep_and_retry:
        k_msleep(SLEEP_TIME_MS);
	}
}
