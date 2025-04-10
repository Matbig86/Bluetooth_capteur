/*
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include <math.h>


// Variables globales
double accel_totale = 0;
double distance = 0;
double vitesse = 0; // Vitesse cumulée
double previous_time = 0;
uint32_t current_time = 0;
uint32_t previous_timestamp = 0;
int cnt = 0;

#ifdef CONFIG_LIS2DW12_TRIGGER
static int lis2dw12_trig_cnt;

static void lis2dw12_trigger_handler(const struct device *dev,
				     const struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	lis2dw12_trig_cnt++;
}
#endif

#ifdef CONFIG_LSM6DSO_TRIGGER
static int lsm6dso_acc_trig_cnt;
static int lsm6dso_gyr_trig_cnt;
static int lsm6dso_temp_trig_cnt;

static void lsm6dso_acc_trig_handler(const struct device *dev,
				     const struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	lsm6dso_acc_trig_cnt++;
}

static void lsm6dso_gyr_trig_handler(const struct device *dev,
				     const struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	lsm6dso_gyr_trig_cnt++;
}

static void lsm6dso_temp_trig_handler(const struct device *dev,
				      const struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_DIE_TEMP);
	lsm6dso_temp_trig_cnt++;
}
#endif


static void lis2mdl_config(const struct device *lis2mdl)
{
	struct sensor_value odr_attr;

	/* set LIS2MDL sampling frequency to 100 Hz */
	odr_attr.val1 = 100;
	odr_attr.val2 = 0;   

	if (sensor_attr_set(lis2mdl, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LIS2MDL\n");
		return;
	}

}


static void lis2dw12_config(const struct device *lis2dw12)
{
	struct sensor_value odr_attr, fs_attr;

	/* set LIS2DW12 accel/gyro sampling frequency to 100 Hz */
	odr_attr.val1 = 100;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lis2dw12, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LIS2DW12 accel\n");
		return;
	}

	sensor_g_to_ms2(16, &fs_attr);

	if (sensor_attr_set(lis2dw12, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set sampling frequency for LIS2DW12 gyro\n");
		return;
	}

#ifdef CONFIG_LIS2DW12_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(lis2dw12, &trig, lis2dw12_trigger_handler);
#endif
}

static void lsm6dso_config(const struct device *lsm6dso)
{
    struct sensor_value odr_attr, fs_attr;

    /* Set LSM6DSO accel sampling frequency to 208 Hz */
    odr_attr.val1 = 208;
    odr_attr.val2 = 0;

    if (sensor_attr_set(lsm6dso, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Cannot set sampling frequency for LSM6DSO accel\n");
        return;
    }

    sensor_g_to_ms2(16, &fs_attr);

    if (sensor_attr_set(lsm6dso, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
        printk("Cannot set full scale for LSM6DSO accel\n");
        return;
    }

    /* Set LSM6DSO gyro sampling frequency to 208 Hz */
    odr_attr.val1 = 208;
    odr_attr.val2 = 0;

    if (sensor_attr_set(lsm6dso, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Cannot set sampling frequency for LSM6DSO gyro\n");
        return;
    }

    sensor_degrees_to_rad(250, &fs_attr);

    if (sensor_attr_set(lsm6dso, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
        printk("Cannot set full scale for LSM6DSO gyro\n");
        return;
    }
}

#ifdef CONFIG_LSM6DSO_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(lsm6dso, &trig, lsm6dso_acc_trig_handler);

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_GYRO_XYZ;
	sensor_trigger_set(lsm6dso, &trig, lsm6dso_gyr_trig_handler);

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_DIE_TEMP;
	sensor_trigger_set(lsm6dso, &trig, lsm6dso_temp_trig_handler);
#endif
