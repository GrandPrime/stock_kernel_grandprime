/*
 * Copyright (C) 2013 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include "sensors_core.h"

#define VENDOR_NAME			"AKM"
#define MODEL_NAME			"AK09911C"
#define MODULE_NAME			"magnetic_sensor"

/* Rx buffer size. i.e ST,TMPS,H1X,H1Y,H1Z*/
#define SENSOR_DATA_SIZE		9
#define AK09911C_DEFAULT_DELAY		200
#define AK09911C_DRDY_TIMEOUT_MS	100
#define AK09911C_WIA1_VALUE		0x48
#define AK09911C_WIA2_VALUE		0x05

/* Compass device dependent definition */
#define AK09911C_REG_WIA1			0x00
#define AK09911C_REG_WIA2			0x01
#define AK09911C_REG_INFO1			0x02
#define AK09911C_REG_INFO2			0x03
#define AK09911C_REG_ST1			0x10
#define AK09911C_REG_HXL			0x11
#define AK09911C_REG_HXH			0x12
#define AK09911C_REG_HYL			0x13
#define AK09911C_REG_HYH			0x14
#define AK09911C_REG_HZL			0x15
#define AK09911C_REG_HZH			0x16
#define AK09911C_REG_TMPS			0x17
#define AK09911C_REG_ST2			0x18
#define AK09911C_REG_CNTL1			0x30
#define AK09911C_REG_CNTL2			0x31
#define AK09911C_REG_CNTL3			0x32

#define AK09911C_FUSE_ASAX			0x60
#define AK09911C_FUSE_ASAY			0x61
#define AK09911C_FUSE_ASAZ			0x62

#define AK09911C_MODE_SNG_MEASURE		0x01
#define AK09911C_MODE_SELF_TEST			0x10
#define AK09911C_MODE_FUSE_ACCESS		0x1F
#define AK09911C_MODE_POWERDOWN			0x00
#define AK09911C_RESET_DATA			0x01

#define AK09911C_TOP_LOWER_RIGHT         0
#define AK09911C_TOP_LOWER_LEFT          1
#define AK09911C_TOP_UPPER_LEFT          2
#define AK09911C_TOP_UPPER_RIGHT         3
#define AK09911C_BOTTOM_LOWER_RIGHT      4
#define AK09911C_BOTTOM_LOWER_LEFT       5
#define AK09911C_BOTTOM_UPPER_LEFT       6
#define AK09911C_BOTTOM_UPPER_RIGHT      7

struct ak09911c_platform_data {
	void (*get_pos)(int *);
	int m_rst_n;
	u32 m_rst_n_flags;
	unsigned int irq_gpio;
	u32 irq_gpio_flags;
	const u8 *reg_vdd;
	const u8 *reg_vio;

};

struct ak09911c_v {
	union {
		s16 v[3];
		struct {
			s16 x;
			s16 y;
			s16 z;
		};
	};
};

struct ak09911c_p {
	struct i2c_client *client;
	struct input_dev *input;
	struct device *factory_device;
	struct ak09911c_v magdata;
	struct mutex lock;
	struct delayed_work work;

	atomic_t delay;
	atomic_t enable;

	u8 asa[3];
	int chip_pos;
	int m_rst_n;
};

static int ak09911c_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *buf, unsigned char len)
{
	s32 dummy;

	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, buf);
	if (dummy < 0) {
		pr_err("[SENSOR]: %s - i2c bus read error %d\n",
			__func__, dummy);
		return -EIO;
	}
	return 0;
}

static int ak09911c_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *buf)
{
	s32 dummy;

	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0) {
		pr_err("[SENSOR]: %s - i2c bus read error %d\n",
			__func__, dummy);
		return -EIO;
	}
	*buf = dummy & 0x000000ff;

	return 0;
}

static int ak09911c_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *buf)
{
	s32 dummy;

	dummy = i2c_smbus_write_byte_data(client, reg_addr, *buf);
	if (dummy < 0) {
		pr_err("[SENSOR]: %s - i2c bus write error %d\n",
			__func__, dummy);
		return -EIO;
	}
	return 0;
}

static int ak09911c_ecs_set_mode_power_down(struct ak09911c_p *data)
{
	unsigned char reg;
	int ret;

	reg = AK09911C_MODE_POWERDOWN;
	ret = ak09911c_smbus_write_byte(data->client, AK09911C_REG_CNTL2, &reg);

	return ret;
}

static int ak09911c_ecs_set_mode(struct ak09911c_p *data, char mode)
{
	u8 reg;
	int ret;

	switch (mode & 0x1F) {
	case AK09911C_MODE_SNG_MEASURE:
		reg = AK09911C_MODE_SNG_MEASURE;
		ret = ak09911c_smbus_write_byte(data->client,
				AK09911C_REG_CNTL2, &reg);
		break;
	case AK09911C_MODE_FUSE_ACCESS:
		reg = AK09911C_MODE_FUSE_ACCESS;
		ret = ak09911c_smbus_write_byte(data->client,
				AK09911C_REG_CNTL2, &reg);
		break;
	case AK09911C_MODE_POWERDOWN:
		reg = AK09911C_MODE_SNG_MEASURE;
		ret = ak09911c_ecs_set_mode_power_down(data);
		break;
	case AK09911C_MODE_SELF_TEST:
		reg = AK09911C_MODE_SELF_TEST;
		ret = ak09911c_smbus_write_byte(data->client,
				AK09911C_REG_CNTL2, &reg);
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	/* Wait at least 300us after changing mode. */
	udelay(100);

	return 0;
}

static void ak09911c_reset(struct ak09911c_p *data)
{
	pr_info("[SENSOR]: %s called!\n", __func__);
	gpio_set_value(data->m_rst_n, 0);
	udelay(5);
	gpio_set_value(data->m_rst_n, 1);
	// Device will be accessible 100 us after
	udelay(100);
}

static int ak09911c_read_mag_xyz(struct ak09911c_p *data, struct ak09911c_v *mag)
{
	u8 temp[SENSOR_DATA_SIZE] = {0, };
	int ret = 0, retries = 0;

	mutex_lock(&data->lock);
	ret = ak09911c_ecs_set_mode(data, AK09911C_MODE_SNG_MEASURE);
	if (ret < 0)
		goto exit_i2c_read_err;

retry:
	ret = ak09911c_smbus_read_byte(data->client,
			AK09911C_REG_ST1, &temp[0]);
	if (ret < 0)
		goto exit_i2c_read_err;

	/* Check ST bit */
	if (!(temp[0] & 0x01)) {
		if ((retries++ < 5) && (temp[0] == 0)) {
			mdelay(2);
			goto retry;
		} else {
			ret = -EAGAIN;
			goto exit_i2c_read_fail;
		}
	}

	ret = ak09911c_smbus_read_byte_block(data->client, AK09911C_REG_ST1 + 1,
			&temp[1], SENSOR_DATA_SIZE - 1);
	if (ret < 0)
		goto exit_i2c_read_err;

	/* Check ST2 bit */
	if ((temp[8] & 0x01)) {
		ret = -EAGAIN;
		goto exit_i2c_read_fail;
	}

	mag->x = temp[1] | (temp[2] << 8);
	mag->y = temp[3] | (temp[4] << 8);
	mag->z = temp[5] | (temp[6] << 8);

	remap_sensor_data(mag->v, data->chip_pos);

	goto exit;

exit_i2c_read_fail:
exit_i2c_read_err:
	pr_err("[SENSOR]: %s - failed. ret = %d, ST1 = %u, ST2 = %u\n",
			__func__, ret, temp[0], temp[8]);
exit:
	mutex_unlock(&data->lock);
	return ret;
}

static void ak09911c_work_func(struct work_struct *work)
{
	struct ak09911c_v mag;
	struct ak09911c_p *data = container_of((struct delayed_work *)work,
			struct ak09911c_p, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&data->delay));
	int ret;

	ret = ak09911c_read_mag_xyz(data, &mag);
	if (ret >= 0) {
		input_report_rel(data->input, REL_X, mag.x);
		input_report_rel(data->input, REL_Y, mag.y);
		input_report_rel(data->input, REL_Z, mag.z);
		input_sync(data->input);
		data->magdata = mag;
	}

	schedule_delayed_work(&data->work, delay);
}

static void ak09911c_set_enable(struct ak09911c_p *data, int enable)
{
	int pre_enable = atomic_read(&data->enable);

	if (enable) {
		if (pre_enable == 0) {
			ak09911c_ecs_set_mode(data, AK09911C_MODE_SNG_MEASURE);
			schedule_delayed_work(&data->work,
				msecs_to_jiffies(atomic_read(&data->delay)));
			atomic_set(&data->enable, 1);
		}
	} else {
		if (pre_enable == 1) {
			ak09911c_ecs_set_mode(data, AK09911C_MODE_POWERDOWN);
			cancel_delayed_work_sync(&data->work);
			atomic_set(&data->enable, 0);
		}
	}
}

static ssize_t ak09911c_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ak09911c_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->enable));
}

static ssize_t ak09911c_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u8 enable;
	int ret;
	struct ak09911c_p *data = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 2, &enable);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	pr_info("[SENSOR]: %s - new_value = %u\n", __func__, enable);
	if ((enable == 0) || (enable == 1))
		ak09911c_set_enable(data, (int)enable);

	return size;
}

static ssize_t ak09911c_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ak09911c_p *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->delay));
}

static ssize_t ak09911c_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	int64_t delay;
	struct ak09911c_p *data = dev_get_drvdata(dev);

	ret = kstrtoll(buf, 10, &delay);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	atomic_set(&data->delay, (unsigned int)delay);
	pr_info("[SENSOR]: %s - poll_delay = %lld\n", __func__, delay);

	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		ak09911c_delay_show, ak09911c_delay_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		ak09911c_enable_show, ak09911c_enable_store);

static struct attribute *ak09911c_attributes[] = {
	&dev_attr_poll_delay.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group ak09911c_attribute_group = {
	.attrs = ak09911c_attributes
};

static int ak09911c_selftest(struct ak09911c_p *data, int *sf)
{
	u8 temp[6], reg;
	s16 x, y, z;
	int retry_count = 0;
	int ready_count = 0;
	int ret;
retry:
	mutex_lock(&data->lock);
	/* power down */
	reg = AK09911C_MODE_POWERDOWN;
	ak09911c_smbus_write_byte(data->client, AK09911C_REG_CNTL2, &reg);

	/* read device info */
	ak09911c_smbus_read_byte_block(data->client, AK09911C_REG_WIA1, temp, 2);
	pr_info("[SENSOR]: %s - device id = 0x%x, info = 0x%x\n",
		__func__, temp[0], temp[1]);

	/* start */
	reg = AK09911C_MODE_SELF_TEST;
	ak09911c_smbus_write_byte(data->client, AK09911C_REG_CNTL2, &reg);

	/* wait for data ready */
	while (ready_count < 10) {
		msleep(20);
		ret = ak09911c_smbus_read_byte(data->client,
				AK09911C_REG_ST1, &reg);
		if ((reg == 1) && (ret == 0))
			break;
		ready_count++;
	}

	ak09911c_smbus_read_byte_block(data->client, AK09911C_REG_HXL,
		temp, sizeof(temp));
	mutex_unlock(&data->lock);

	x = temp[0] | (temp[1] << 8);
	y = temp[2] | (temp[3] << 8);
	z = temp[4] | (temp[5] << 8);

	/* Hadj = (H*(Asa+128))/128 */
	x = (x * (data->asa[0] + 128)) >> 7;
	y = (y * (data->asa[1] + 128)) >> 7;
	z = (z * (data->asa[2] + 128)) >> 7;

	pr_info("[SENSOR]: %s - x = %d, y = %d, z = %d\n",
		__func__, x, y, z);
	if ((x >= -30) && (x <= 30))
		pr_info("[SENSOR]: %s - x passed, -30<=x<=30\n", __func__);
	else
		pr_info("[SENSOR]: %s - x failed, -30<=x<=30\n", __func__);
	if ((y >= -30) && (y <= 30))
		pr_info("[SENSOR]: %s - y passed, -30<=y<=30\n", __func__);
	else
		pr_info("[SENSOR]: %s - y failed, -30<=y<=30\n", __func__);
	if ((z >= -400) && (z <= -50))
		pr_info("[SENSOR]: %s - z passed, -400<=z<=-50\n", __func__);
	else
		pr_info("[SENSOR]: %s - z failed, -400<=z<=-50\n", __func__);

	sf[0] = x;
	sf[1] = y;
	sf[2] = z;

	if (((x >= -30) && (x <= 30)) &&
		((y >= -30) && (y <= 30)) &&
		((z >= -400) && (z <= -50))) {
		pr_info("[SENSOR] %s, Selftest is successful.\n", __func__);
		return 1;
	} else {
		if (retry_count < 5) {
			retry_count++;
			pr_warn("############################################");
			pr_warn("%s, retry_count=%d\n", __func__, retry_count);
			pr_warn("############################################");
			goto retry;
		} else {
			pr_err("[SENSOR]: %s - Selftest is failed.\n",
				__func__);
			return 0;
		}
	}
}

static ssize_t ak09911c_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t ak09911c_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", MODEL_NAME);
}

static ssize_t ak09911c_get_asa(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ak09911c_p *data  = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u,%u,%u\n",
			data->asa[0], data->asa[1], data->asa[2]);
}

static ssize_t ak09911c_get_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int sf[3] = {0,};

	ret = ak09911c_selftest(dev_get_drvdata(dev), sf);
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d\n",
			ret, sf[0], sf[1], sf[2]);
}

static ssize_t ak09911c_check_registers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 temp[13], reg;
	struct ak09911c_p *data = dev_get_drvdata(dev);

	mutex_lock(&data->lock);
	/* power down */
	reg = AK09911C_MODE_POWERDOWN;
	ak09911c_smbus_write_byte(data->client, AK09911C_REG_CNTL2, &reg);
	/* get the value */
	ak09911c_smbus_read_byte_block(data->client, AK09911C_REG_WIA1, temp, 13);

	mutex_unlock(&data->lock);

	return snprintf(buf, PAGE_SIZE,
			"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			temp[0], temp[1], temp[2], temp[3], temp[4], temp[5],
			temp[6], temp[7], temp[8], temp[9], temp[10], temp[11],
			temp[12]);
}

static ssize_t ak09911c_check_cntl(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 reg;
	int ret = 0;
	struct ak09911c_p *data = dev_get_drvdata(dev);

	mutex_lock(&data->lock);
	/* power down */
	reg = AK09911C_MODE_POWERDOWN;

	ret = ak09911c_smbus_write_byte(data->client, AK09911C_REG_CNTL2, &reg);
	udelay(100);
	ret += ak09911c_smbus_read_byte(data->client, AK09911C_REG_CNTL2, &reg);
	mutex_unlock(&data->lock);

	return snprintf(buf, PAGE_SIZE, "%s\n",
			(((reg == AK09911C_MODE_POWERDOWN) &&
			(ret == 0)) ? "OK" : "NG"));
}

static ssize_t ak09911c_get_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	bool success;
	struct ak09911c_p *data = dev_get_drvdata(dev);

	if ((data->asa[0] == 0) | (data->asa[0] == 0xff)
		| (data->asa[1] == 0) | (data->asa[1] == 0xff)
		| (data->asa[2] == 0) | (data->asa[2] == 0xff))
		success = false;
	else
		success = true;

	return snprintf(buf, PAGE_SIZE, "%s\n", (success ? "OK" : "NG"));
}

static ssize_t ak09911c_adc(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	bool success = false;
	int ret;
	struct ak09911c_p *data = dev_get_drvdata(dev);
	struct ak09911c_v mag = data->magdata;

	if (atomic_read(&data->enable) == 1) {
		success = true;
		msleep(20);
		goto exit;
	}

	ret = ak09911c_read_mag_xyz(data, &mag);
	if (ret < 0)
		success = false;
	else
		success = true;

exit:
	return snprintf(buf, PAGE_SIZE, "%s,%d,%d,%d\n",
			(success ? "OK" : "NG"), mag.x, mag.y, mag.z);
}

static ssize_t ak09911c_raw_data_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ak09911c_p *data = dev_get_drvdata(dev);
	struct ak09911c_v mag = data->magdata;

	if (atomic_read(&data->enable) == 1) {
		msleep(20);
		goto exit;
	}

	ak09911c_read_mag_xyz(data, &mag);

exit:
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", mag.x, mag.y, mag.z);
}

static DEVICE_ATTR(name, S_IRUGO, ak09911c_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, ak09911c_vendor_show, NULL);
static DEVICE_ATTR(raw_data, S_IRUGO, ak09911c_raw_data_read, NULL);
static DEVICE_ATTR(adc, S_IRUGO, ak09911c_adc, NULL);
static DEVICE_ATTR(dac, S_IRUGO, ak09911c_check_cntl, NULL);
static DEVICE_ATTR(chk_registers, S_IRUGO, ak09911c_check_registers, NULL);
static DEVICE_ATTR(selftest, S_IRUGO, ak09911c_get_selftest, NULL);
static DEVICE_ATTR(asa, S_IRUGO, ak09911c_get_asa, NULL);
static DEVICE_ATTR(status, S_IRUGO, ak09911c_get_status, NULL);

static struct device_attribute *sensor_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_raw_data,
	&dev_attr_adc,
	&dev_attr_dac,
	&dev_attr_chk_registers,
	&dev_attr_selftest,
	&dev_attr_asa,
	&dev_attr_status,
	NULL,
};

static int ak09911c_read_fuserom(struct ak09911c_p *data)
{
	unsigned char reg;
	int ret;

	/* put into fuse access mode to read asa data */
	reg = AK09911C_MODE_FUSE_ACCESS;
	ret = ak09911c_smbus_write_byte(data->client, AK09911C_REG_CNTL2, &reg);
	if (ret < 0) {
		pr_err("[SENSOR]: %s - unable to enter fuse rom mode\n",
			__func__);
		goto exit_default_value;
	}

	ret = ak09911c_smbus_read_byte_block(data->client, AK09911C_FUSE_ASAX,
			data->asa, sizeof(data->asa));
	if (ret < 0) {
		pr_err("[SENSOR]: %s - unable to load factory sensitivity "\
			"adjust values\n", __func__);
		goto exit_default_value;
	} else
		pr_info("[SENSOR]: %s - asa_x = %u, asa_y = %u, asa_z = %u\n",
			__func__, data->asa[0], data->asa[1], data->asa[2]);

	reg = AK09911C_MODE_POWERDOWN;
	ret = ak09911c_smbus_write_byte(data->client, AK09911C_REG_CNTL2, &reg);
	if (ret < 0)
		pr_err("[SENSOR] Error in setting power down mode\n");

	return 0;

exit_default_value:
	data->asa[0] = 0;
	data->asa[1] = 0;
	data->asa[2] = 0;

	return ret;
}

static int ak09911c_check_device(struct ak09911c_p *data)
{
	unsigned char reg, buf[2];
	int ret;

	ret = ak09911c_smbus_read_byte_block(data->client,
			AK09911C_REG_WIA1, buf, 2);
	if (ret < 0) {
		pr_err("[SENSOR]: %s - unable to read AK09911C_REG_WIA1\n",
			__func__);
		return ret;
	}

	reg = AK09911C_MODE_POWERDOWN;
	ret = ak09911c_smbus_write_byte(data->client, AK09911C_REG_CNTL2, &reg);
	if (ret < 0) {
		pr_err("[SENSOR]: %s - Error in setting power down mode\n",
			__func__);
		return ret;
	}

	if ((buf[0] != AK09911C_WIA1_VALUE) || (buf[1] != AK09911C_WIA2_VALUE)) {
		pr_err("[SENSOR]: %s - The device is not AKM Compass. %u, %u",
			__func__, buf[0], buf[1]);
		return -ENXIO;
	}

	return 0;
}

static int ak09911c_setup_pin(struct ak09911c_platform_data *pdata)
{
	int ret;

	ret = gpio_request(pdata->m_rst_n, "M_RST_N");
	if (ret < 0) {
		pr_err("[SENSOR] %s - gpio %d request failed (%d)\n",
			__func__, pdata->m_rst_n, ret);
		goto exit;
	}

	ret = gpio_direction_output(pdata->m_rst_n, 1);
	if (ret < 0) {
		pr_err("[SENSOR]: %s - failed to set gpio %d as input (%d)\n",
			__func__, pdata->m_rst_n, ret);
		goto exit_reset_gpio;
	}
	gpio_set_value(pdata->m_rst_n, 1);

	goto exit;

exit_reset_gpio:
	gpio_free(pdata->m_rst_n);
exit:
	return ret;
}

static int ak09911c_input_init(struct ak09911c_p *data)
{
	int ret = 0;
	struct input_dev *dev;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = MODULE_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, REL_X);
	input_set_capability(dev, EV_REL, REL_Y);
	input_set_capability(dev, EV_REL, REL_Z);
	input_set_drvdata(dev, data);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

	ret = sensors_create_symlink(&dev->dev.kobj, dev->name);
	if (ret < 0) {
		input_unregister_device(dev);
		return ret;
	}

	/* sysfs node creation */
	ret = sysfs_create_group(&dev->dev.kobj, &ak09911c_attribute_group);
	if (ret < 0) {
		sensors_remove_symlink(&data->input->dev.kobj,
			data->input->name);
		input_unregister_device(dev);
		return ret;
	}

	data->input = dev;
	return 0;
}

static int ak09911c_parse_dt(struct device *dev,struct ak09911c_p *data,
	struct ak09911c_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	if (pdata == NULL)
		return -ENODEV;

	pdata->m_rst_n = of_get_named_gpio_flags(np, "ak09911c-i2c,m_rst_n",
	0, &pdata->m_rst_n_flags);
	pdata->reg_vdd = of_get_property(np,"ak09911c-i2c,reg_vdd", NULL);
	pdata->reg_vio = of_get_property(np,"ak09911c-i2c,reg_vio", NULL);

	if (pdata->get_pos != NULL)
		pdata->get_pos(&data->chip_pos);
	else
		data->chip_pos = AK09911C_TOP_UPPER_RIGHT;

		printk("[SENSOR]: %s -chip_pos: %d\n", __func__,data->chip_pos);

	if (pdata->m_rst_n < 0) {
		pr_err("[SENSOR]: %s - m_rst_n error\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int ak09911c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct ak09911c_p *data = NULL;
	struct ak09911c_platform_data *pdata;

	pr_info("[SENSOR]: %s - Probe Start!\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[SENSOR]: %s - i2c_check_functionality error\n",
			__func__);
		goto exit;
	}

	data = kzalloc(sizeof(struct ak09911c_p), GFP_KERNEL);
	if (data == NULL) {
		pr_err("[SENSOR]: %s - kzalloc error\n", __func__);
		ret = -ENOMEM;
		goto exit_kzalloc;
	}
	if (client-> dev.of_node) {
		pdata = devm_kzalloc (&client->dev ,
			sizeof (struct ak09911c_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = ak09911c_parse_dt(&client->dev,data, pdata);
		if (ret < 0) {
			pr_err("[SENSOR]: %s - of_node error\n", __func__);
			ret = -ENODEV;
			goto exit_of_node;
			}

		} else
			pdata = client->dev.platform_data;
		if (!pdata)
			return -EINVAL;

	ret = ak09911c_setup_pin(pdata);
	if (ret) {
		pr_err("[SENSOR]: %s - could not setup pin\n", __func__);
		goto exit_setup_pin;
	}

	/* ak09911c chip reset */
	ak09911c_reset(data);

	i2c_set_clientdata(client, data);
	data->client = client;

	ret = ak09911c_check_device(data);
	if (ret < 0) {
		pr_err("[SENSOR]: %s - ak09911c_check_device fail"\
			"(err=%d)\n", __func__, ret);
		goto exit_set_mode_check_device;
	}

	/* input device init */
	ret = ak09911c_input_init(data);
	if (ret < 0)
		goto exit_input_init;

	sensors_register(data->factory_device, data, sensor_attrs, MODULE_NAME);

	/* workqueue init */
	INIT_DELAYED_WORK(&data->work, ak09911c_work_func);
	mutex_init(&data->lock);

	atomic_set(&data->delay, AK09911C_DEFAULT_DELAY);
	atomic_set(&data->enable, 0);

	ak09911c_read_fuserom(data);

	pr_info("[SENSOR]: %s - Probe done!(chip pos : %d)\n",
		__func__, data->chip_pos);

	return 0;

exit_input_init:
exit_set_mode_check_device:
	gpio_free(data->m_rst_n);
exit_setup_pin:
exit_of_node:
	kfree(data);
exit_kzalloc:
exit:
	pr_err("[SENSOR]: %s - Probe fail!\n", __func__);
	return ret;
}

static void ak09911c_shutdown(struct i2c_client *client)
{
	struct ak09911c_p *data = (struct ak09911c_p *)i2c_get_clientdata(client);

	pr_info("[SENSOR]: %s\n", __func__);
	if (atomic_read(&data->enable) == 1) {
		ak09911c_ecs_set_mode(data, AK09911C_MODE_POWERDOWN);
		cancel_delayed_work_sync(&data->work);
	}
}

static int ak09911c_remove(struct i2c_client *client)
{
	struct ak09911c_p *data = (struct ak09911c_p *)i2c_get_clientdata(client);

	if (atomic_read(&data->enable) == 1)
		ak09911c_set_enable(data, 0);

	gpio_free(data->m_rst_n);
	mutex_destroy(&data->lock);

	sensors_unregister(data->factory_device, sensor_attrs);
	sensors_remove_symlink(&data->input->dev.kobj, data->input->name);

	sysfs_remove_group(&data->input->dev.kobj, &ak09911c_attribute_group);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

static int ak09911c_suspend(struct device *dev)
{
	struct ak09911c_p *data = dev_get_drvdata(dev);

	if (atomic_read(&data->enable) == 1) {
		ak09911c_ecs_set_mode(data, AK09911C_MODE_POWERDOWN);
		cancel_delayed_work_sync(&data->work);
	}

	return 0;
}

static int ak09911c_resume(struct device *dev)
{
	struct ak09911c_p *data = dev_get_drvdata(dev);

	if (atomic_read(&data->enable) == 1) {
		ak09911c_ecs_set_mode(data, AK09911C_MODE_SNG_MEASURE);
		schedule_delayed_work(&data->work,
		msecs_to_jiffies(atomic_read(&data->delay)));
	}

	return 0;
}

static const struct i2c_device_id ak09911c_id[] = {
	{ "ak09911c-i2c", 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id magnetic_match_table[] = {
	{.compatible = "magnetic,ak09911c-i2c",},
	{},
};
#else
#define magnetic_match_table NULL
#endif

static const struct dev_pm_ops ak09911c_pm_ops = {
	.suspend = ak09911c_suspend,
	.resume = ak09911c_resume
};

static struct i2c_driver ak09911c_driver = {
	.driver = {
		.name	= MODEL_NAME,
		.owner	= THIS_MODULE,
		.pm = &ak09911c_pm_ops,
		.of_match_table = magnetic_match_table,
	},
	.probe		= ak09911c_probe,
	.shutdown	= ak09911c_shutdown,
	.remove		= ak09911c_remove,
	.id_table	= ak09911c_id,
};

static int __init ak09911c_init(void)
{
	return i2c_add_driver(&ak09911c_driver);
}

static void __exit ak09911c_exit(void)
{
	i2c_del_driver(&ak09911c_driver);
}

module_init(ak09911c_init);
module_exit(ak09911c_exit);

MODULE_DESCRIPTION("AK09911C compass driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
