/*
 * DSPG DBMD2 codec driver
 *
 * Copyright (C) 2014 DSP Group
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#include <linux/vmalloc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "sbl.h"
#include <linux/input.h>
#include "dbmd2-export.h"
#include <linux/platform_data/msm_serial_hs.h>

/* Real buffer size is 96000 samples, i.e. ~6 seconds; to be safe use 5 seconds */
#define MAX_AUDIO_BUFFER_SIZE		(160000) /* 5 seconds */
/* Size must be power of 2 */
#define MAX_KFIFO_BUFFER_SIZE		(131072*2) /* >4 seconds */

#define MAX_AMODEL_SIZE			(128*1024)

struct dbd2_platform_data {
	unsigned gpio_wakeup;
	unsigned gpio_reset;
	unsigned gpio_sensory;
	int sensory_irq;
};

/* To support sysfs node  */
static struct class *ns_class;
static struct device *dbd2_dev, *gram_dev, *net_dev;
static void dbd2_firmware_ready(const struct firmware *fw, void *context);

#define DRIVER_VERSION			"1.003"
#define DBD2_FIRMWARE_NAME		"dbd2_fw.bin"
#define DBD2_MODEL_NAME			"A-Model.bin"
#define DBD2_GRAM_NAME			"UDT_Always_Deep_search.bin" //"gram.bin"
#define DBD2_NET_NAME			"UDT_Always_Deep_recog.bin" //"net.bin"
#define MAX_CMD_SEND_SIZE		32 //1024
#define RETRY_COUNT			5

/* DBD2 commands and values */

#define DBD2_SYNC_POLLING		0x80000000

#define DBD2_SET_POWER_STATE_SLEEP	0x80100001

#define DBD2_GET_FW_VER			0x80000000
#define DBD2_OPR_MODE			0x80010000
#define DBD2_TG_THERSHOLD		0x80020000
#define DBD2_VERIFICATION_THRESHOLD	0x80030000
#define DBD2_GAIN_SHIFT_VALUE		0x80040000
#define DBD2_IO_PORT_ADDR_LO		0x80050000
#define DBD2_IO_PORT_ADDR_HI		0x80060000
#define DBD2_IO_PORT_VALUE_LO		0x80070000
#define DBD2_IO_PORT_VALUE_HI		0x80080000
#define DBD2_AUDIO_BUFFER_SIZE		0x80090000
#define DBD2_NUM_OF_SMP_IN_BUF		0x800A0000
#define DBD2_LAST_MAX_SMP_VALUE		0x800B0000
#define DBD2_LAST_DETECT_WORD_NUM	0x800C0000
#define DBD2_DETECT_TRIGER_LEVEL	0x800D0000
#define DBD2_DETECT_VERIFICATION_LEVEL	0x800E0000
#define DBD2_LOAD_NEW_ACUSTIC_MODEL	0x800F0000
#define DBD2_UART_SPEED			0x80100000
#define DBD2_UART_XON			0x80110000
#define DBD2_AUDIO_BUFFER_CONVERSION	0x80120000
#define DBD2_UART_XOFF			0x80130000
#define DBD2_LAST_DURATION		0x80140000
#define DBD2_LAST_ERROR			0x80150000
#define DBD2_MIC_GAIN			0x80160000
#define DBD2_FW_ID			0x80190000
#define DBD2_BUFFERING_BACKLOG_SIZE	0x801B0000
#define DBD2_POST_TRIGGER_AUDIO_BUF	0x80200000

#define DBD2_SET_D2PARAM_ADDR		0x801C0000
#define DBD2_GET_D2PARAM		0x80270000
#define DBD2_SET_D2PARAM		0x80260000

#define DBD2_READ_CHECKSUM		0x805A0E00
#define DBD2_FIRMWARE_BOOT		0x805A0B00

#define DBD2_8KHZ			0x0008

#define DBD2_AUDIO_MODE_PCM		0
#define DBD2_AUDIO_MODE_MU_LAW		1

#define FW_BOOT_RESPONSE_SIZE		28
#define UART_TTY_WRITE_SZ		8
#define UART_TTY_READ_SZ		512 //32
#define UART_TTY_BAUD_RATE		57600
#define UART_TTY_BOOT_BAUD_RATE		115200
#define UART_TTY_3S_DOWNLOAD_BAUD_RATE	460800
#define UART_TTY_MAX_BAUD_RATE		3000000
#define UART_TTY_STOP_BITS		1
#define UART_TTY_BOOT_STOP_BITS		2
#define UART_TTY_MAX_EAGAIN_RETRY	20
#define UART_TTY_MAX_HW_BUF_SIZE	8192
struct dbd2_data {
	struct dbd2_platform_data	pdata;
	struct platform_device		*pdev;
	struct device			*dev;
	struct i2c_client		*client;
	const struct firmware		*fw;
	struct mutex			lock;
//	struct mutex			lock_data_transfer;
	bool				asleep;
	bool				device_ready;
	bool				change_speed;
	struct clk			*clk;
	struct work_struct		sensory_work;
	struct work_struct		uevent_work;
	unsigned int			audio_buffer_size;
	unsigned int			audio_mode;
	unsigned int			bytes_per_sample;
	dev_t				record_chrdev;
	struct cdev			record_cdev;
	struct device			*record_dev;
	int				audio_processed;
	char				*amodel_fw_name;
	struct tty_struct		*uart_tty;
	struct file			*uart_file;
	struct tty_ldisc		*uart_ld;
	const char			*uart_dev;
	int				uart_open;
	struct kfifo			pcm_kfifo;
	int				a_model_loaded;
	atomic_t			audio_owner;
	int				auto_buffering;
	int				buffering;
	struct input_dev		*input;
	char				*amodel_buf;
	int				amodel_len;
	char				*gram_data, *net_data;
	int				gram_size, net_size;
	int				detection_state;

	struct firmware			*dspg_gram;
	struct firmware			*dspg_net;

	struct uart_port		*uport;
};

struct dbd2_data *dbd2_data;

static int dbd2_send_cmd(struct dbd2_data *dbd2, u32 command, u16 *response);
static int dbd2_send_cmd_short(struct dbd2_data *dbd2, u32 command, u16 *response);
static int dbmd2_common_probe(struct dbd2_data *dbd2);
static int dbmd2_uart_read(struct dbd2_data *dbd2, u8 *buf, int len);
//static int dbmd2_uart_read_sync(struct dbd2_data *dbd2, u8 *buf, int len);
static int dbmd2_uart_write_sync(struct dbd2_data *dbd2, const u8 *buf, int len);
static void dbmd2_uart_close_file(struct dbd2_data *dbd2);
static int dbmd2_uart_open_file(struct dbd2_data *dbd2);
static int dbmd2_uart_configure_tty(struct tty_struct *tty, u32 bps, int stop, int parity, int flow);
static int dbd2_wait_till_alive(struct dbd2_data *dbd2);
static int dbmd2_read_data(struct dbd2_data *dbd2, unsigned int bytes_to_read);
static int dbd2_set_uart_speed(struct dbd2_data *dbd2, int index);
static int dbd2_set_uart_speed1(struct dbd2_data *dbd2, int index);


#define uart_lock(x) \
mutex_lock(x); \
dbd2_uart_clk_enable(1);

#define uart_unlock(x) \
dbd2_uart_clk_enable(0);\
mutex_unlock(x); 

static void
dbd2_clk_enable(bool enable)
{
	/* TODO */
	//printk("24MHz clock %s\n", enable? "on": "off");
	int rc;
	printk("%s start (%s)\n", __func__, enable?"ON":"OFF");
	dbd2_data->clk = clk_get(dbd2_data->dev, "dbd2_clk");
	if(enable){
		clk_set_rate(dbd2_data->clk, 2000000);
		rc = clk_prepare_enable(dbd2_data->clk);
		if(rc < 0)
			printk("%s: clk_prepare_enable failed\n", __func__);
	} else {
		clk_disable_unprepare(dbd2_data->clk);
		clk_put(dbd2_data->clk);
	}
}

static void
dbd2_uart_clk_enable(bool enable)
{
	if (!dbd2_data)
		return;

	if (enable) {
		msm_hs_request_clock_on(dbd2_data->uport);
		msleep(20);
	} else {
		msm_hs_request_clock_off(dbd2_data->uport);
	}
}

static void
dbd2_flush_rx_fifo(struct dbd2_data *dbd2)
{
#define __SIZE		16
	int ret;
	char resp[__SIZE];
//	loff_t pos = 0;

	if (dbd2->client)
		return;

	/* empty fifo */
	do {
#if 0		
//		ret = vfs_read(dbd2->uart_file, (char __user *)&resp[0], __SIZE, &pos);
		ret = dbd2->uart_ld->ops->read(dbd2->uart_tty,
					       dbd2->uart_file,
					       (char __user *)resp, __SIZE);
	} while (ret > 0);
#endif
		ret = dbmd2_uart_read(dbd2, (char *)&resp, __SIZE);
	} while (ret > 0);

#undef __SIZE
}

static const char *uart_speed_text[] = {
	"57600",
	"460800",
	"3000000",
};

static const unsigned int uart_speed[] = {
	57600,
	460800,
	3000000,
};

enum dbd2_uart_speeds {
	DBD2_UART_SPEED_57600 = 0,
	DBD2_UART_SPEED_460800,
	DBD2_UART_SPEED_3000000,
	DBD2_UART_SPEEDS,
};

static int
dbd2_buf_to_int(const char *buf)
{
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	return (int)val;
}

static int
dbd2_set_bytes_per_sample(struct dbd2_data *dbd2, unsigned int mode)
{
	int ret;

	/* Changing the buffer conversion causes trouble: adapting the
	 * UART baudrate doesn't work anymore (firmware bug?) */
	if (((mode == DBD2_AUDIO_MODE_PCM) && (dbd2->bytes_per_sample == 2)) ||
	    ((mode == DBD2_AUDIO_MODE_MU_LAW) && (dbd2->bytes_per_sample == 1)))
		return 0;

	ret = dbd2_send_cmd_short(dbd2,
				  DBD2_AUDIO_BUFFER_CONVERSION | mode,
				  NULL);
	if (ret < 0) {
		dev_err(dbd2->dev, "failed to set DBD2_AUDIO_BUFFER_CONVERSION\n");
		return ret;
	}

	switch (mode) {
	case DBD2_AUDIO_MODE_PCM:
		dbd2->bytes_per_sample = 2;
		break;
	case DBD2_AUDIO_MODE_MU_LAW:
		dbd2->bytes_per_sample = 1;
		break;
	default:
		break;
	}

	return 0;
}

static int
dbd2_sleeping(struct dbd2_data *dbd2)
{
	return dbd2->asleep;
}

static int
dbd2_wake(struct dbd2_data *dbd2)
{
	int ret = 0;

	/* if chip not sleeping there is nothing to do */
	if (!dbd2_sleeping(dbd2))
		return 0;

	/* deassert wake pin */
	/*	gpio_set_value(pdata->gpio_wakeup, 0); */
	/* give some time to wakeup */
	//msleep(30);

	/* test if firmware is up */
	ret = dbd2_send_cmd(dbd2, DBD2_SYNC_POLLING, NULL);
	if (ret < 0) {
		dev_err(dbd2->dev, "sync error\n");
		/* assert gpio pin */
		/*	gpio_set_value(pdata->gpio_wakeup, 1); */
		return -EIO;
	}

	/* make it not sleeping */
	dbd2->asleep = false;

	dev_info(dbd2->dev, "%s: wake up\n", __func__);
	return 0;
}

enum dbmd2_states {
	DBMD2_IDLE = 0,
	DBMD2_DETECTION,
	DBMD2_RESERVED,
	DBMD2_BUFFERING,
	DBMD2_SLEEP,
	DBMD2_HIBERNATE,
};

static int
dbd2_get_mode(struct dbd2_data *dbd2)
{
	int ret;
	u16 state;

	ret = dbd2_send_cmd_short(dbd2, DBD2_OPR_MODE, &state);
	if (ret < 0) {
		dev_err(dbd2->dev, "failed to read DBD2_OPR_MODE\n");
		return ret;
	}

	return (int)state;
}

static int
dbd2_set_mode(struct dbd2_data *dbd2, int mode)
{
	int ret;

	printk("%s: mode(%d)\n", __func__, mode);
	/* set new mode and return old one */

	/* nothing to do */
	if (dbd2_sleeping(dbd2) && mode == DBMD2_HIBERNATE)
		return DBMD2_HIBERNATE;

	/* wakeup chip */
	ret = dbd2_wake(dbd2);
	if (ret) {
		return -EIO;
	}

	/* anything special to do */
	switch (mode) {
	case DBMD2_HIBERNATE:
		dbd2->asleep = true;
		break;
	case DBMD2_IDLE:
		break;
	case DBMD2_BUFFERING:
		/* set higher speed */
		if (!dbd2->client) {
			ret = dbd2_set_uart_speed(dbd2, DBD2_UART_SPEED_460800);
			if (ret)
				dev_err(dbd2->dev, "failed switch to higher speed\n");
		}
		break;
	case DBMD2_DETECTION:
		break;
	case DBMD2_SLEEP:
		break;
	default:
		break;
	}

	/* set operation mode register */
	ret = dbd2_send_cmd(dbd2, 0x80000000 | (0x01 << 16) |
			    (mode & 0xffff), NULL);
	if (ret < 0)
		dev_err(dbd2->dev, "failed to set mode 0x%x\n", mode);

	if (mode == DBMD2_BUFFERING) {
		dbd2->buffering = 1;
		schedule_work(&dbd2->sensory_work);
	}

	return 0;
}

static int
dbd2_wait_till_alive(struct dbd2_data *dbd2)
{
	u16 result;
	int ret = 0;
	int state;
	int timeout = 2000000; /* 2s timeout */
	unsigned long stimeout = jiffies + msecs_to_jiffies(200);

	msleep(100);

	if (!dbd2->client) {
		dbd2_flush_rx_fifo(dbd2);

		/* Poll to wait for firmware completing its wakeup procedure:
		 * Read the firmware ID number (0xdbd2) */
		do {
			ret = dbd2_send_cmd_short(dbd2, DBD2_FW_ID, &result);
			if (ret < 0) {
				//dev_err(dbd2->dev, "failed to read firmware id\n");
				continue;
			}
			if (result == 0xdbd2) {
				ret = 0;
			} else
				ret = -1;
		} while (time_before(jiffies, stimeout) && ret != 0);
		if (ret != 0)
			dev_err(dbd2->dev, "failed to read firmware id\n");
		ret = (ret >= 0? 1: 0);
		if (!ret)
			dev_err(dbd2->dev, "%s(): failed = 0x%d\n", __func__, ret);
	} else {
		do {
			state = dbd2_get_mode(dbd2);
			/* wait around 50ms if state not changed */
			/* XXX reconsider this */
			return 1;
			if (state < 0) {
				usleep_range(50000, 51000);
				timeout -= 50000;
			}
		} while (state < 0 && timeout > 0);
		ret = (state < 0? 0: 1);
	}

	return ret;
}

static int
dbmd2_uart_read(struct dbd2_data *dbd2, u8 *buf, int len)
{
	int ret;
	mm_segment_t oldfs;
	loff_t pos = 0;
	int retry = UART_TTY_MAX_EAGAIN_RETRY;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	do {
		ret = vfs_read(dbd2->uart_file,
				(char __user *)buf, len, &pos);
		if (ret == -EAGAIN) {
			usleep_range(2000, 2000);
		}
		retry--;
	} while (ret == -EAGAIN && retry);

	set_fs(oldfs);

//	if (ret < 0)
//		dev_err(dbd2->dev, "%s(): read error %d\n", __func__, ret);
	return ret;
}

static int
dbmd2_uart_write(struct dbd2_data *dbd2, const u8 *buf, int len)
{
#if 1
	int ret = 0;
	int count_remain = len;
	int bytes_wr = 0;
	mm_segment_t oldfs;

	/* we may call from user context via char dev, so allow
	 * read buffer in kernel address space */
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	while (count_remain > 0) {
		while (tty_write_room(dbd2->uart_tty) < UART_TTY_WRITE_SZ)
			usleep_range(2000, 2000);

		ret = dbd2->uart_ld->ops->write(dbd2->uart_tty,
					        dbd2->uart_file,
					        (char __user *)buf + bytes_wr,
					        min(UART_TTY_WRITE_SZ, count_remain));
		if (ret < 0) {
			bytes_wr = ret;
			goto err_out;
		}

		bytes_wr += ret;
		count_remain -= ret;
	}

err_out:
	set_fs(oldfs);

	return bytes_wr;
#else
	int ret = 0;
	int count_remain = len;
	int bytes_wr = 0;
	mm_segment_t oldfs;
	loff_t pos = 0;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	while (count_remain > 0) {
		while (tty_write_room(dbd2->uart_tty) < UART_TTY_WRITE_SZ)
			usleep_range(2000, 2000);

		ret = vfs_write(dbd2->uart_file, buf + bytes_wr,
				min(UART_TTY_WRITE_SZ, count_remain), &pos);

		if (ret < 0) {
			bytes_wr = ret;
			goto err_out;
		}

		bytes_wr += ret;
		count_remain -= ret;
	}

err_out:
	set_fs(oldfs);

	return bytes_wr;
#endif
}

static int
dbmd2_uart_read_sync(struct dbd2_data *dbd2, u8 *buf, int len)
{
#if 1
	mm_segment_t oldfs;
	int rc;
	int i = 0;
	int bytes_to_read = len;
	unsigned long timeout = jiffies + msecs_to_jiffies(2000);

	/* we may call from user context via char dev, so allow
	 * read buffer in kernel address space */
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	do {
		rc = dbd2->uart_ld->ops->read(dbd2->uart_tty,
					       dbd2->uart_file,
					       (char __user *)buf + i, bytes_to_read);
		if (rc <= 0)
			continue;
		bytes_to_read -= rc;
		i += rc;
	} while (time_before(jiffies, timeout) && bytes_to_read);

	/* restore old fs context */
	set_fs(oldfs);

	if (bytes_to_read)
		return -EIO;

	return len;
#else
	int counter = 0;
	int rc;
	int i = 0;
	int bytes_to_read = len;
	unsigned long timeout = jiffies + msecs_to_jiffies(2000);

	do {
		rc = dbmd2_uart_read(dbd2, buf + i, bytes_to_read);
		if (rc == -EAGAIN)
			counter++;		
		if (rc <= 0)
			continue;
		bytes_to_read -= rc;
		i += rc;
	} while (time_before(jiffies, timeout) && bytes_to_read);
	if (bytes_to_read)
		printk("dbmd2_uart_read_sync: %d\n", counter);
	if (bytes_to_read)
		return -EIO;

	return len;
#endif
}
/*
static int
dbmd2_uart_read_async(struct dbd2_data *dbd2, u8 *buf, int len)
{
	int rc;
	int i = 0;
	int bytes_to_read = len;

	do {
		rc = dbmd2_uart_read(dbd2, buf + i, bytes_to_read);
		if (rc <= 0)
			break;
		bytes_to_read -= rc;
		i += rc;
	} while (bytes_to_read);

	if (bytes_to_read) {
		printk("bytes to read: %d/%d\n", bytes_to_read, len);
		return -EIO;
	}

	return len;
}
*/

static int
dbmd2_uart_write_sync(struct dbd2_data *dbd2, const u8 *buf, int len)
{
	int ret = 0;
	int count_remain = len;
	int bytes_wr = 0;
	mm_segment_t oldfs;

	/* we may call from user context via char dev, so allow
	 * read buffer in kernel address space */
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	while (count_remain > 0) {
		/* block until tx buffer space is available */
		while (tty_write_room(dbd2->uart_tty) < UART_TTY_WRITE_SZ)
			usleep_range(5000, 5000);

		ret = dbd2->uart_ld->ops->write(dbd2->uart_tty,
						dbd2->uart_file,
						buf + bytes_wr,
						min(UART_TTY_WRITE_SZ,
						count_remain));

		if (ret < 0) {
			bytes_wr = ret;
			goto err_out;
		}

		bytes_wr += ret;
		count_remain -= ret;
	}

err_out:
	/* restore old fs context */
	set_fs(oldfs);

	return bytes_wr;
}


static int
dbmd2_uart_configure_tty(struct tty_struct *tty, u32 bps, int stop, int parity, int flow)
{
	int rc = 0;
	struct ktermios termios;

	memcpy(&termios, &(tty->termios), sizeof(termios));

	tty_wait_until_sent(tty,0);

	termios.c_cflag &= ~(CBAUD | CSIZE | PARENB | CSTOPB);   /* clear csize, baud */
	termios.c_cflag |= BOTHER; /* allow arbitrary baud */
	termios.c_cflag |= CS8;
	if ( parity) {
		termios.c_cflag |=  PARENB ;
	}
	if (stop == 2)
		termios.c_cflag |= CSTOPB;

	/* set uart port to raw mode (see termios man page for flags) */
	termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
		| INLCR | IGNCR | ICRNL | IXON | IXOFF);

	if (flow)
		termios.c_iflag |= IXOFF;      /* enable XON/OFF for input */

	termios.c_oflag &= ~(OPOST);
	termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	/* set baud rate */
	termios.c_ospeed = bps;
	termios.c_ispeed = bps;

	rc = tty_set_termios(tty, &termios);

	//dev_dbg(dbd2->dev, "%s(): New baud %u\n", __func__,
	//	tty->termios->c_ospeed);

	return rc;
}

static int
dbd2_set_uart_speed1(struct dbd2_data *dbd2, int index)
{
	int rvalue = -EIO;
	int ret;
	int state = DBMD2_IDLE;
//	char resp;

	if (dbd2->client)
		return 0;

	ret = dbd2_send_cmd(dbd2, DBD2_UART_SPEED | index, NULL);
	if (ret < 0) {
		dev_err(dbd2->dev, "could not set new uart speed\n");
		goto out_change_mode;
	}

#if 0
	dbmd2_uart_read_sync(dbd2, (char *)&resp, 1);
	if (resp != 'U') {
		dev_err(dbd2->dev, "firmware did not respond to uart speed change\n");
		goto out_change_mode;
	}
#endif

	msleep(10); /* FIXME */

//	ssleep(1);
//	dev_info(dbd2->dev, "configure tty\n");
	/* set baudrate to FW baud (common case) */
	dbmd2_uart_configure_tty(dbd2->uart_tty,
				 uart_speed[index],
				 UART_TTY_STOP_BITS,
				 0, 1);

	dbd2_flush_rx_fifo(dbd2);
	rvalue = 0;
	goto out;
out_change_mode:
	if (index > DBD2_UART_SPEED_460800) {
		/* leave sleep mode */
		ret = dbd2_set_mode(dbd2, state);
		if (ret) {
			dev_err(dbd2->dev, "failed to set old mode\n");
			goto out;
		}
	}
	dbd2_flush_rx_fifo(dbd2);
out:
	return rvalue;
}


static int
dbd2_set_uart_speed(struct dbd2_data *dbd2, int index)
{
	int rvalue = -EIO;
	int ret;
	int state = DBMD2_IDLE;
//	char resp;

	if (dbd2->client)
		return 0;

	if (index > DBD2_UART_SPEED_460800) {
		/* A speed of 3Mbaud available only in sleep mode */
		state = dbd2_get_mode(dbd2);
		if (state < 0) {
			dev_err(dbd2->dev, "device not responding\n");
			goto out;
		}
		ret = dbd2_set_mode(dbd2, DBMD2_SLEEP);
		if (ret) {
			dev_err(dbd2->dev, "failed to set device to sleep mode\n");
			goto out;
		}
	}

	ret = dbd2_send_cmd(dbd2, DBD2_UART_SPEED | index, NULL);
	if (ret < 0) {
		dev_err(dbd2->dev, "could not set new uart speed\n");
		goto out_change_mode;
	}

#if 0
	dbmd2_uart_read_sync(dbd2, (char *)&resp, 1);
	if (resp != 'U') {
		dev_err(dbd2->dev, "firmware did not respond to uart speed change\n");
		goto out_change_mode;
	}
#endif

	msleep(10); /* FIXME */

//	ssleep(1);
//	dev_info(dbd2->dev, "configure tty\n");
	/* set baudrate to FW baud (common case) */
	dbmd2_uart_configure_tty(dbd2->uart_tty,
				 uart_speed[index],
				 UART_TTY_STOP_BITS,
				 0, 1);

	dbd2_flush_rx_fifo(dbd2);
//	dev_info(dbd2->dev, "configure tty done\n");

//	dev_info(dbd2->dev, "wait till alive\n");
	ret = dbd2_wait_till_alive(dbd2);
	if (!ret) {
		dev_err(dbd2->dev, "device not responding\n");
		goto out;
	}
//	ssleep(1);
//	dev_info(dbd2->dev, "wait till alive done\n");
	rvalue = 0;
	goto out;
out_change_mode:
	if (index > DBD2_UART_SPEED_460800) {
		/* leave sleep mode */
		ret = dbd2_set_mode(dbd2, state);
		if (ret) {
			dev_err(dbd2->dev, "failed to set old mode\n");
			goto out;
		}
	}
	dbd2_flush_rx_fifo(dbd2);
out:
	return rvalue;
}

static void
dbmd2_uart_close_file(struct dbd2_data *dbd2)
{
	if (dbd2->uart_open) {
		filp_close(dbd2->uart_file, 0);
		dbd2->uart_open = 0;
	}
}

static int
dbmd2_uart_open_file(struct dbd2_data *dbd2)
{
	long err = 0;
	struct file *fp;
	int attempt = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(60000);

//	mutex_lock(&dbd2->lock);

	if (dbd2->uart_open)
		goto out_unlock;

	/* Wait for the device node to appear in the filesystem. This can take
	 * some time if the kernel is still booting up and filesystems are
	 * being mounted. */
	do {
		msleep(50);
		dev_dbg(dbd2->dev,
			"%s(): probing for tty on %s (attempt %d)\n",
			 __func__, dbd2->uart_dev, ++attempt);

		fp = filp_open(dbd2->uart_dev, O_RDWR | O_NONBLOCK | O_NOCTTY, 0);

		err = PTR_ERR(fp);
	} while (time_before(jiffies, timeout) && err == -ENOENT);

	if (IS_ERR_OR_NULL(fp)) {
		dev_err(dbd2->dev, "UART device node open failed\n");
		err = -ENODEV;
		goto out_unlock;
	}

	/* set uart_dev members */
	dbd2->uart_file = fp;
	dbd2->uart_tty = ((struct tty_file_private *)fp->private_data)->tty;
	dbd2->uart_ld = tty_ldisc_ref(dbd2->uart_tty);

	dbd2->uart_open = 1;
	err = 0;

	dbd2->uport = msm_hs_get_uart_port(0);

out_unlock:
//	mutex_unlock(&dbd2->lock);
	return err;
}

static int
dbmd2_uart_open_thread(void *data)
{
	int ret;
	struct dbd2_data *dbd2 = (struct dbd2_data *)data;

	ret = dbmd2_uart_open_file(dbd2);
	if (ret)
		return ret;

	return dbmd2_common_probe(dbd2);
}

static int
dbmd2_uart_open(struct dbd2_data *dbd2)
{
	int rc = 0;
	struct task_struct *uart_probe_thread = NULL;

	dev_dbg(dbd2->dev, "%s():\n", __func__);
	uart_probe_thread = kthread_run(dbmd2_uart_open_thread,
					(void *)dbd2,
					"dbmd2 probe thread");
	if (IS_ERR_OR_NULL(uart_probe_thread)) {
		pr_err("%s(): can't create dbmd2 uart probe thread = %p\n",
			__func__, uart_probe_thread);
		rc = -ENOMEM;
	}

	return rc;
	/* return dbmd2_common_probe(dbd2); */
}

static void
dbmd2_uart_close(struct dbd2_data *dbd2)
{
	tty_ldisc_deref(dbd2->uart_ld);
	dbmd2_uart_close_file(dbd2);
}

static int
dbd2_send_i2c_cmd(struct dbd2_data *dbd2, u32 command, u16 *response)
{
	u8 send[4];
	u8 recv[4];
	int ret = 0;
	int retry = RETRY_COUNT;

	send[0] = dbd2->client->addr /*(command >> 24)*/ & 0xff;
	send[1] = (command >> 16) & 0xff;
	send[2] = (command >> 8) & 0xff;
	send[3] = command & 0xff;

	ret = i2c_master_send(dbd2->client, send, 4);
	if (ret < 0) {
		dev_err(dbd2->dev, "i2c_master_send failed ret = %d\n", ret);
		return ret;
	}

	/* The sleep command cannot be acked before the device goes to sleep */
	if (command == DBD2_SET_POWER_STATE_SLEEP)
		return ret;
	else if (command == DBD2_SYNC_POLLING)
		usleep_range(1000, 2000);
	/* A host command received will blocked until the current audio frame
	   processing is finished, which can take up to 10 ms */
	else
		usleep_range(10000, 11000);

	if (response) {
		while (retry--) {
			ret = i2c_master_recv(dbd2->client, recv, 4);
			if (ret < 0) {
				dev_err(dbd2->dev, "i2c_master_recv failed\n");
				return ret;
			}
			/*
			 * Check that the first two bytes of the response match
			 * (the ack is in those bytes)
			 */
			if ((send[0] == recv[0]) && (send[1] == recv[1])) {
				if (response)
					*response = (recv[2] << 8) | recv[3];
				ret = 0;
				break;
			} else {
				dev_err(dbd2->dev,
					"incorrect ack (got 0x%.2x%.2x)\n",
					recv[0], recv[1]);
				ret = -EINVAL;
			}

			/* Wait before polling again */
			if (retry > 0)
				msleep(20);
		}
	}

	return ret;
}

static int
dbd2_send_uart_cmd(struct dbd2_data *dbd2, u32 command, u16 *response)
{
	char tmp[3];
	u8 send[7];
	u8 recv[6];
	int ret = 0;

	if (response)
		dbd2_flush_rx_fifo(dbd2);

	ret = snprintf(tmp, 3, "%02x", (command >> 16) & 0xff);
	if (ret < 0) {
//		dev_err(dbd2->dev, "dbmd2_send_uart_cmd: invalid command\n");
		return ret;
	}
	send[0] = tmp[0];
	send[1] = tmp[1];
	send[2] = 'w';

	ret = snprintf(tmp, 3, "%02x", (command >> 8) & 0xff);
	if (ret < 0) {
//		dev_err(dbd2->dev, "dbmd2_send_uart_cmd: invalid command\n");
		return ret;
	}
	send[3] = tmp[0];
	send[4] = tmp[1];

	ret = snprintf(tmp, 3, "%02x", command & 0xff);
	if (ret < 0) {
//		dev_err(dbd2->dev, "dbmd2_send_uart_cmd: invalid command\n");
		return ret;
	}
	send[5] = tmp[0];
	send[6] = tmp[1];

	ret = dbmd2_uart_write(dbd2, send, 7);
	if (ret < 0) {
//		dev_err(dbd2->dev, "dbmd2_uart_write failed ret = %d\n", ret);
		return ret;
	}

	/* The sleep command cannot be acked before the device goes to sleep */
	if (command == DBD2_SET_POWER_STATE_SLEEP)
		return ret;
	/* A host command received will blocked until the current audio frame
	   processing is finished, which can take up to 10 ms */
	usleep_range(10000, 11000);

	if (response) {
		ret = dbmd2_uart_read_sync(dbd2, recv, 5);
		if (ret < 0) {
//			dev_err(dbd2->dev, "dbmd2_uart_read failed\n");
			return ret;
		}
		recv[5] = 0;
		ret = kstrtou16(recv, 16, response);
		if (ret < 0) {
			dev_err(dbd2->dev, "dbmd2_uart_read failed\n");
			dev_err(dbd2->dev, "%x:%x:%x:%x:\n", recv[0], recv[1], recv[2], recv[3]);
			return ret;
		}
	}

	return ret;
}

static int
dbd2_send_cmd(struct dbd2_data *dbd2, u32 command, u16 *response)
{
	if (dbd2->client)
		return dbd2_send_i2c_cmd(dbd2, command, response);
	else if (!dbd2->client)
		return dbd2_send_uart_cmd(dbd2, command, response);

	return -ENODEV;
}

static int
dbd2_send_i2c_cmd_short(struct dbd2_data *dbd2, u32 command, u16 *response)
{
	u8 send[2];
	u8 recv[2];
	int ret = 0;

	send[0] = dbd2->client->addr /*(command >> 24)*/ & 0xff;
	send[1] = (command >> 16) & 0xff;

	ret = i2c_master_send(dbd2->client, send, 2);
	if (ret < 0) {
		dev_err(dbd2->dev, "i2c_master_send failed ret = %d\n", ret);
		return ret;
	}

	/* The sleep command cannot be acked before the device goes to sleep */
	if (command == DBD2_SET_POWER_STATE_SLEEP)
		return ret;
	/* A host command received will blocked until the current audio frame
	   processing is finished, which can take up to 10 ms */
	usleep_range(10000, 11000);

	if (response) {
		ret = i2c_master_recv(dbd2->client, recv, 2);
		if (ret < 0) {
			dev_err(dbd2->dev, "i2c_master_recv failed\n");
			return ret;
		}
		memcpy(response, recv, 2);
		ret = 0;
	}

	return ret;
}

static int
dbd2_send_i2c_cmd_boot(struct dbd2_data *dbd2, u32 command)
{
	u8 send[3];
	int ret = 0;

	send[0] = dbd2->client->addr /*(command >> 24)*/ & 0xff;
	send[1] = (command >> 16) & 0xff;
	send[2] = (command >>  8) & 0xff;

	ret = i2c_master_send(dbd2->client, send, 3);
	if (ret < 0) {
		dev_err(dbd2->dev, "i2c_master_send failed ret = %d\n", ret);
		return ret;
	}

	/* A host command received will blocked until the current audio frame
	   processing is finished, which can take up to 10 ms */
	usleep_range(10000, 11000);

	return ret;
}

static int
dbd2_send_uart_cmd_short(struct dbd2_data *dbd2, u32 command, u16 *response)
{
	char tmp[3];
	u8 send[3];
	u8 recv[6];
	int ret = 0;

	if (response)
		dbd2_flush_rx_fifo(dbd2);

	ret = snprintf(tmp, 3, "%02x", (command >> 16) & 0xff);
	send[0] = tmp[0];
	send[1] = tmp[1];
	send[2] = 'r';

	ret = dbmd2_uart_write(dbd2, send, 3);
	if (ret < 0) {
//		dev_err(dbd2->dev, "dbmd2_uart_write failed ret = %d\n", ret);
		return ret;
	}

	/* The sleep command cannot be acked before the device goes to sleep */
	if (command == DBD2_SET_POWER_STATE_SLEEP)
		return ret;
	/* A host command received will blocked until the current audio frame
	   processing is finished, which can take up to 10 ms */
	usleep_range(10000, 11000);

	if (response) {
		ret = dbmd2_uart_read_sync(dbd2, recv, 5);
		if (ret < 0) {
//			dev_err(dbd2->dev, "dbmd2_uart_read failed\n");
			return ret;
		}
		recv[5] = 0;
		ret = kstrtou16(recv, 16, response);
		if (ret < 0) {
			dev_err(dbd2->dev, "dbmd2_uart_read conversion failed\n");
			dev_err(dbd2->dev, "%x:%x:%x:%x:\n", recv[0], recv[1], recv[2], recv[3]);
			return ret;
		}
	}

	return ret;
}

static int
dbd2_send_cmd_short(struct dbd2_data *dbd2, u32 command, u16 *response)
{
	if (dbd2->client)
		return dbd2_send_i2c_cmd_short(dbd2, command, response);
	else if (!dbd2->client)
		return dbd2_send_uart_cmd_short(dbd2, command, response);

	return -ENODEV;
}

static int
dbd2_send_i2c_data(struct dbd2_data *dbd2, const u8 *data, size_t size)
{
	int ret = 0;
	const u8 *i2c_cmds = data;
	int to_copy = size;

	while (to_copy > 0) {
		ret = i2c_master_send(dbd2->client, i2c_cmds,
				min(to_copy, MAX_CMD_SEND_SIZE));
		if (ret < 0) {
			dev_err(dbd2->dev, "i2c_master_send failed ret=%d\n",
				ret);
			break;
		}
		to_copy -= MAX_CMD_SEND_SIZE;
		i2c_cmds += MAX_CMD_SEND_SIZE;
	}

	return ret;
}

static int
dbd2_send_data(struct dbd2_data *dbd2, const u8 *data, size_t size)
{
	if (dbd2->client)
		return dbd2_send_i2c_data(dbd2, data, size);
	else if (!dbd2->client)
		return dbmd2_uart_write_sync(dbd2, data, size);

	return -ENODEV;
}

static int
dbmd2_uart_sync(struct dbd2_data *dbmd2)
{
	int rc;
	unsigned long timeout = jiffies + msecs_to_jiffies(100);
	char resp[4] = {'1','1','1','1'};
	char match[] = "OK\n\r";
	u16 *buf;
	int i;
	/* Send init sequence for up to 100ms at 115200baud.
	 * 1 start bit, 8 data bits, 1 parity bit, 2 stop bits = 12 bits */
	unsigned int size = UART_TTY_BOOT_BAUD_RATE / 120;

	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		dev_err(dbmd2->dev, "no memory for sync buffer\n");
		return -ENOMEM;
	}

	for (i = 0; i < (size / sizeof(u16)); i++)
		buf[i] = 0x6341;

	dbmd2_uart_write_sync(dbmd2, (char *)buf, size);

	kfree(buf);

	/* DBMD2P2 answers with "OK\n\r" */
	do {
		rc = dbmd2_uart_read(dbmd2, (char *)&resp[0], 1);
	} while (time_before(jiffies, timeout) && resp[0] != 'O');

	if (resp[0] == 'O') {
		dbmd2_uart_read_sync(dbmd2, (char *)&resp[1], 3);
		rc = strncmp(resp, match, 4);
	} else {
		rc = -1;
	}
//	printk("dbmd2 2: ---- %x:%x:%x:%x(%d)\n", resp[0], resp[1], resp[2], resp[3], rc);
	return rc;
}

static int
dbmd2_wait_for_ok(struct dbd2_data *dbd2)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	char resp[2];
	char match[] = "OK";
	int rc = 0;

	do {
		rc = dbmd2_uart_read(dbd2, (char *)&resp[0], 1);
	} while (time_before(jiffies, timeout) && resp[0] != 'O');

	if (resp[0] == 'O') {
		dbmd2_uart_read_sync(dbd2, (char *)&resp[1], 1);
		rc = strncmp(resp, match, 2);
	} else {
		rc = -1;
	}
	return rc;
}

static int
dbd2_verify_checksum(struct dbd2_data *dbd2, const char *fw_checksum)
{
	int ret = 0;
	char rx_checksum[6];
	u8 cmd[2] = {0x5a, 0x0e};

	if (!dbd2->client) {
		ret = dbmd2_uart_write(dbd2, (char *)&cmd, 2);
		if (ret < 0)
			return -1;
		dbmd2_uart_read_sync(dbd2, (char *)&rx_checksum[0], 6);
	} else {
		ret = dbd2_send_i2c_cmd_boot(dbd2, DBD2_READ_CHECKSUM);
		if (ret < 0) {
			dev_err(dbd2->dev, "could not read checksum\n");
			return -1;
		}

		ret = i2c_master_recv(dbd2->client, rx_checksum, 6);
		if (ret < 0) {
			dev_err(dbd2->dev, "could not read checksum data\n");
			return -1;
		}
	}
	/* printk("checksum: 0x%x 0x%x 0x%x 0x%x\n", rx_checksum[2], rx_checksum[3],
		rx_checksum[4], rx_checksum[5]); */
	return memcmp(fw_checksum, (void *)&rx_checksum[2], 4);
}

static unsigned long
dbd2_calc_amodel_checksum(const char *amodel, unsigned long len)
{
	unsigned long chksum = 0;
	unsigned short val;
	unsigned long i, pos = 0, chunk_len;

	while (pos < len) {
		val = *(unsigned short *)(&amodel[pos]);
		pos += 2;

		if (val == 0x025a) {
			chksum += 0x5a + 0x02;

			chunk_len = *(unsigned long *)(&amodel[pos]);
			pos += 4;

			chksum += chunk_len;

			chksum += *(unsigned long *)(&amodel[pos]);
			pos += 4;

			for (i = 0; i < chunk_len; i++) {
				chksum += *(unsigned short *)(&amodel[pos]);
				pos += 2;
			}
		} else return -1;
	}

	chksum += 0x5A + 0x0e;

	return chksum;
}

static int
dbd2_boot_firmware(struct dbd2_data *dbd2)
{
	int ret;
	u8 bcmd[2] = {0x5a, 0x0b};

	if (!dbd2->client) {
		ret = dbmd2_uart_write(dbd2, (char *)&bcmd, 2);
		if (ret < 0)
			return -1;
	} else {
		ret = dbd2_send_i2c_cmd_boot(dbd2, DBD2_FIRMWARE_BOOT);
		if (ret < 0) {
			dev_err(dbd2->dev, "could not boot firmware\n");
			return -1;
		}
	}
	return 0;
}

static void
dbd2_get_firmware_version(const char *data, size_t size, char *version)
{
	int i, j;

	version[0] = 0;
	for (i = size - 13; i > 0; i--) {
		if ((data[i]   == 'v') && (data[i+2]  == 'e') &&
		    (data[i+4] == 'r') && (data[i+6]  == 's') &&
		    (data[i+8] == 'i') && (data[i+10] == 'o')) {
			for (j = 0; i + j < size; j++) {
				version[j] = data[i];
				i += 2;
				if (((version[j] > 0) && (version[j] < 32))
				    || (version[j] > 126))
					return;
				if (version[j] == 0) version[j] = ' ';
			}
			version[j] = 0;
			return;
		}
	}
}

static int
dbd2_reset(struct dbd2_data *dbd2)
{
	int ret = 0;
	struct dbd2_platform_data *pdata = &dbd2->pdata;
	int retry = RETRY_COUNT;
	const char *fw_checksum;
	char fw_version[100];

	if (dbd2->fw->size < 4)
		return -1;

	fw_checksum = &dbd2->fw->data[dbd2->fw->size - 4];
	dbd2_get_firmware_version(dbd2->fw->data, dbd2->fw->size, fw_version);
	if (strlen(fw_version) > 0)
		dev_info(dbd2->dev, "firmware %s\n", fw_version);

	if (!dbd2->client) {
		/* set baudrate to FW baud (common case) */
		dbmd2_uart_configure_tty(dbd2->uart_tty,
					 UART_TTY_BOOT_BAUD_RATE,
					 UART_TTY_BOOT_STOP_BITS,
					 1, 0);
	}

	while (retry--) {
		/* Reset DBD2 chip */
		gpio_set_value(pdata->gpio_reset, 0);
		usleep_range(200, 300);
		gpio_set_value(pdata->gpio_reset, 1);

		/* Delay before sending commands */
		usleep_range(20000, 30000);

		if (!dbd2->client) {
			dbd2_flush_rx_fifo(dbd2);

			ret = dbmd2_uart_sync(dbd2);
			if (ret != 0) {
				dev_err(dbd2->dev, "sync failed, retry\n");
				continue;
			}

			dbd2_flush_rx_fifo(dbd2);

			/* sbl */
			ret = dbd2_send_data(dbd2, sbl, sizeof(sbl));
			if (ret < 0) {
				dev_err(dbd2->dev, "---------> load sbl error\n");
				continue;
			}

			/* check if sbl is ok */
			ret = dbmd2_wait_for_ok(dbd2);
			if (ret != 0) {
				dev_err(dbd2->dev, "sbl does not respond with ok\n");
				continue;
			}

			/* give sbl time to settle with max baudrate */
			msleep(10);

			/* set baudrate to FW baud (common case) */
			dbmd2_uart_configure_tty(dbd2->uart_tty,
						 UART_TTY_MAX_BAUD_RATE,
						 UART_TTY_STOP_BITS,
						 0, 0);
			dbd2_flush_rx_fifo(dbd2);
		}

		/* send firmware */
		ret = dbd2_send_data(dbd2, dbd2->fw->data, dbd2->fw->size - 4);
		if (ret < 0) {
			dev_err(dbd2->dev, "-----------> load firmware error\n");
			continue;
		}

		if (dbd2->client)
			msleep(50);

		/* verify checksum */
		ret = dbd2_verify_checksum(dbd2, fw_checksum);
		if (ret != 0) {
			dev_err(dbd2->dev, "-----------> load firmware checksum error\n");
			continue;
		}

		dev_dbg(dbd2->dev, "---------> firmware loaded\n");
		break;
	}

	ret = dbd2_boot_firmware(dbd2);
	if (ret) {
		dev_err(dbd2->dev, "booting the firmware failed\n");
		return -1;
	}

	/* FIXME: wait some time till the bytes went out */
	msleep(10);

	if (!dbd2->client) {
		/* set baudrate to FW baud (common case) */
		dbmd2_uart_configure_tty(dbd2->uart_tty,
					 UART_TTY_BAUD_RATE,
					 UART_TTY_STOP_BITS,
					 0, 1);
		dbd2_flush_rx_fifo(dbd2);
	}

	return (retry < 0? -1: 0);
}

static void
dbd2_acoustic_model_load(struct dbd2_data *dbd2, const u8 *data, size_t size)
{
	int ret;
	u16 result;
	int retry = RETRY_COUNT;
	//unsigned long checksum;
	// const char *checksum;

	flush_work(&dbd2_data->sensory_work);

	//mutex_lock(&dbd2->lock_data_transfer);
	uart_lock(&dbd2->lock);

	dbd2->device_ready = false;

	printk("%s\n", __func__);

	/* wakeup chip */
	dbd2_wake(dbd2);

	/* set chip to idle mode */
	ret = dbd2_set_mode(dbd2, DBMD2_IDLE);
	if (ret) {
		dev_err(dbd2->dev, "failed to set device to idle mode\n");
		goto out_unlock;
	}

	if (dbd2->change_speed) {
		/* enable high speed clock */
//		dbd2_clk_enable(1);

		ret = dbd2_set_uart_speed1(dbd2, DBD2_UART_SPEED_460800);
		if (ret != 0) {
			dev_err(dbd2->dev, "failed to change UART speed to highspeed\n");
			goto out_unlock;
		}
	}

	// msleep(10);
	while (retry--) {
		ret = dbd2_send_cmd(dbd2, DBD2_LOAD_NEW_ACUSTIC_MODEL, NULL);
		if (ret < 0) {
			dev_err(dbd2->dev, "failed to set firmware to recieve new acoustic model\n");
			goto out_change_speed;
		}

		dev_info(dbd2->dev, "---------> acoustic model download start\n");
		ret = dbd2_send_data(dbd2, data, size); // - 4);
		// -4 in case checksum is appended
		if (ret < 0) {
			dev_err(dbd2->dev, "sending of acoustic model data failed\n");
			goto out_change_speed;
		}
		dev_info(dbd2->dev, "---------> acoustic model download done\n");

/*		checksum = *(unsigned long *)(&data[size - 4]);
		if (dbd2_verify_checksum(dbd2, (const char *)&checksum)) {
			dev_err(dbd2->dev, "checksum of A-model failed\n");
			continue;
		}
*/
		break;
	}

	ret = dbd2_boot_firmware(dbd2);
	if (ret) {
		dev_err(dbd2->dev, "booting the firmware failed\n");
//		mutex_unlock(&dbd2->lock);
		goto out_change_speed;
	}

	/* set previous values */
	(void)dbd2_send_cmd(dbd2, DBD2_AUDIO_BUFFER_SIZE | dbd2->audio_buffer_size/8, NULL);
	dbd2_set_bytes_per_sample(dbd2, dbd2->audio_mode);

	ret = dbd2_send_cmd_short(dbd2, DBD2_FW_ID, &result);
	if (ret < 0) {
		dev_err(dbd2->dev, "failed to read firmware id\n");
	}
	if (result == 0xdbd2) {
		dev_info(dbd2->dev, "acoustic model send\n");
		dbd2->device_ready = true;
		dbd2->a_model_loaded = 1;
		ret = 0;
	} else {
		dev_info(dbd2->dev, "acoustic model send failed\n");
		ret = -1;
	}

	dbd2_set_mode(dbd2, DBMD2_DETECTION);

out_change_speed:
	if (dbd2->change_speed) {
		dbd2->change_speed = 0;
		ret = dbd2_set_uart_speed(dbd2, DBD2_UART_SPEED_57600);
		if (ret != 0)
			dev_err(dbd2->dev, "failed to change UART speed to normal\n");

		/* disable high speed clock */
//		dbd2_clk_enable(0);
	}
out_unlock:
	
	uart_unlock(&dbd2->lock);
	//mutex_unlock(&dbd2->lock_data_transfer);
}

static int
dbd2_acoustic_model_build(struct dbd2_data *dbd2)
{
	unsigned char head[10] = { 0 };
	int pos, checksum;

	pos = 0;

	head[0] = 0x5A;
	head[1] = 0x02;
	head[2] =  (dbd2->gram_size/2)        & 0xff;
	head[3] = ((dbd2->gram_size/2) >>  8) & 0xff;
	head[4] = ((dbd2->gram_size/2) >> 16) & 0xff;
	head[5] = ((dbd2->gram_size/2) >> 24) & 0xff;
	head[7] = 0x18;
	memcpy(dbd2->amodel_buf, head, 10);

	pos += 10;

	if (pos + dbd2->gram_size > MAX_AMODEL_SIZE)
		return -1;

	memcpy(dbd2->amodel_buf + pos, dbd2->gram_data, dbd2->gram_size);

	pos += dbd2->gram_size;

	head[0] = 0x5A;
	head[1] = 0x02;
	head[2] =  (dbd2->net_size/2)        & 0xff;
	head[3] = ((dbd2->net_size/2) >>  8) & 0xff;
	head[4] = ((dbd2->net_size/2) >> 16) & 0xff;
	head[5] = ((dbd2->net_size/2) >> 24) & 0xff;
	head[7] = 0x1b;
	memcpy(dbd2->amodel_buf + pos, head, 10);

	pos += 10;

	if (pos + dbd2->net_size + 6 > MAX_AMODEL_SIZE)
		return -1;

	memcpy(dbd2->amodel_buf + pos, dbd2->net_data, dbd2->net_size);

	checksum = dbd2_calc_amodel_checksum((char *)dbd2->amodel_buf,
					     pos + dbd2->net_size);
	*(unsigned long *)(dbd2->amodel_buf + pos + dbd2->net_size) = checksum;

	return pos + dbd2->net_size; // + 4;
}
/*
static void
dbd2_acoustic_model_ready(const struct firmware *fw, void *context)
{
	struct dbd2_data *dbd2 = (struct dbd2_data *)context;

	if (!fw)
		return;

	dbd2_acoustic_model_load(dbd2, fw->data, fw->size);

	release_firmware(fw);
}
*/
static void
dbd2_gram_bin_ready(const struct firmware *fw, void *context)
{
	struct dbd2_data *dbd2 = (struct dbd2_data *)context;
	int amodel_size = 0;

	printk("%s\n", __func__);

	if (!fw)
		return;

	printk("%s after fw check\n", __func__);

	mutex_lock(&dbd2->lock);
	dbd2->gram_data = vmalloc(fw->size);
	if (!dbd2->gram_data){
		mutex_unlock(&dbd2->lock);
		return;
	}
	memcpy(dbd2->gram_data, fw->data, fw->size);
	dbd2->gram_size = fw->size;
	if (dbd2->net_data && dbd2->net_size) {
		amodel_size = dbd2_acoustic_model_build(dbd2);
		vfree(dbd2->gram_data);
		vfree(dbd2->net_data);
		dbd2->gram_data = NULL;
		dbd2->net_data = NULL;
	}
	mutex_unlock(&dbd2->lock);

	printk("%s: amodel_size = %d\n", __func__, amodel_size);

	if (amodel_size > 0)
		dbd2_acoustic_model_load(dbd2, dbd2->amodel_buf, amodel_size);

	release_firmware(fw);
}

static void
dbd2_net_bin_ready(const struct firmware *fw, void *context)
{
	struct dbd2_data *dbd2 = (struct dbd2_data *)context;
	int amodel_size = 0;

	printk("%s\n", __func__);

	if (!fw)
		return;

	printk("%s after fw check\n", __func__);

	mutex_lock(&dbd2->lock);
	dbd2->net_data = vmalloc(fw->size);
	if (!dbd2->net_data){
		mutex_unlock(&dbd2->lock);
		return;
	}
	memcpy(dbd2->net_data, fw->data, fw->size);
	dbd2->net_size = fw->size;
	if (dbd2->gram_data && dbd2->gram_size) {
		amodel_size = dbd2_acoustic_model_build(dbd2);
		vfree(dbd2->gram_data);
		vfree(dbd2->net_data);
		dbd2->gram_data = NULL;
		dbd2->net_data = NULL;
	}
	mutex_unlock(&dbd2->lock);

	printk("%s: amodel_size = %d\n", __func__, amodel_size);

	if (amodel_size > 0)
		dbd2_acoustic_model_load(dbd2, dbd2->amodel_buf, amodel_size);

	release_firmware(fw);
}

/* ------------------------------------------------------------------------
 * sysfs attributes
 * ------------------------------------------------------------------------ */
static ssize_t
dbd2_reg_show(struct device *dev, u32 command, struct device_attribute *attr,
	      char *buf)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	int ret;
	u16 val = 0;

	uart_lock(&dbd2->lock);
#if 0
	ret = dbd2_wake(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to wake\n");
		uart_unlock(&dbd2->lock);
		return ret;
	}
#endif
	ret = dbd2_send_cmd_short(dbd2, command, &val);
	if (ret < 0) {
		dev_err(dbd2->dev, "get reg %x error\n",command);
		goto out_unlock;
	}

	ret = sprintf(buf, "0x%x\n", val);
out_unlock:
	
	uart_unlock(&dbd2->lock);
	return ret;
}

static ssize_t
dbd2_reg_show_long(struct device *dev, u32 command, u32 command1,
		   struct device_attribute *attr, char *buf)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	int ret;
	u16 val = 0;
        u32 result;

	uart_lock(&dbd2->lock);
#if 0
	ret = dbd2_wake(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to wake\n");
		uart_unlock(&dbd2->lock);
		return ret;
	}
#endif
	ret = dbd2_send_cmd_short(dbd2, command1, &val);
	if (ret < 0) {
		dev_err(dbd2->dev, "get reg %u error\n", command);
		goto out_unlock;
	}

	dev_err(dbd2->dev, "dbd2_reg_show_long = val = %d\n", val);
	result = (u32)(val & 0xffff);
	val = 0;
	ret = dbd2_send_cmd_short(dbd2, command, &val);
	if (ret < 0) {
		dev_err(dbd2->dev, "get reg %u error\n", command1);
		goto out_unlock;
	}

	dev_err(dbd2->dev, "dbd2_reg_show_long = val = %d\n", val);

	result += ((u32)val << 16) ;

	dev_err(dbd2->dev, "dbd2_reg_show_long = val = %d\n", result);

	ret = sprintf(buf, "0x%x\n", result);
out_unlock:
	
	uart_unlock(&dbd2->lock);
	return ret;
}

static ssize_t
dbd2_reg_store(struct device *dev,u32 command, struct device_attribute *attr,
	       const char *buf, size_t size)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	/* XXX */
	if (!dbd2->device_ready)
		return -EAGAIN;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	uart_lock(&dbd2->lock);

	ret = dbd2_wake(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to wake\n");
		size = ret;
		goto out_unlock;
	}

	if (command == DBD2_OPR_MODE) {
		ret = dbd2_set_mode(dbd2, val);
		if (ret)
			size = ret;
		goto out_unlock;
	}

	if (command == DBD2_AUDIO_BUFFER_CONVERSION) {
		ret = dbd2_set_bytes_per_sample(dbd2, val);
		if (ret)
			size = ret;
		goto out_unlock;
	}

	ret = dbd2_send_cmd(dbd2, command | (u32)val, NULL);
	if (ret < 0) {
		dev_err(dbd2->dev, "set reg error\n");
		size = ret;
		goto out_unlock;
	}

out_unlock:
	
	uart_unlock(&dbd2->lock);
	return size;
}

static ssize_t
dbd2_reg_store_long(struct device *dev, u32 command, u32 command1,
		    struct device_attribute *attr, const char *buf,
		    size_t size)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	if (!dbd2->device_ready)
		return -EAGAIN;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;
	dev_err(dbd2->dev, "dbd2_reg_store_long  val = %u\n", (int)val);

	uart_lock(&dbd2->lock);
	ret = dbd2_wake(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to wake\n");
		size = ret;
		goto out_unlock;
	}

	ret = dbd2_send_cmd(dbd2, command1 | (val & 0xffff), NULL);
	if (ret < 0) {
		dev_err(dbd2->dev, "set reg error\n");
		size = ret;
		goto out_unlock;
	}

	ret = dbd2_send_cmd(dbd2, command | (val >> 16), NULL);
	if (ret < 0) {
		dev_err(dbd2->dev, "set reg error\n");
		size = ret;
		goto out_unlock;
	}

out_unlock:
	
	uart_unlock(&dbd2->lock);
	return size;
}

static ssize_t
dbd2_fw_ver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return dbd2_reg_show(dev, DBD2_GET_FW_VER, attr, buf);
}

static ssize_t
dbd2_opr_mode_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return dbd2_reg_show(dev, DBD2_OPR_MODE, attr, buf);
}

static ssize_t
dbd2_opr_mode_store(struct device *dev, struct device_attribute *attr,
		    const char *buf, size_t size)
{
	return dbd2_reg_store(dev, DBD2_OPR_MODE, attr, buf, size);
}

static ssize_t
dbd2_uart_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	int ret;

	ret = dbmd2_uart_open_file(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "UART initialization failed\n");
		return ret;
	}

	request_firmware_nowait(THIS_MODULE,
				FW_ACTION_HOTPLUG,
				DBD2_FIRMWARE_NAME,
				dbd2->dev,
				GFP_KERNEL,
				dbd2,
				dbd2_firmware_ready);

	return size;
}

static ssize_t
dbd2_trigger_level_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return dbd2_reg_show(dev, DBD2_TG_THERSHOLD, attr, buf);
}

static ssize_t
dbd2_trigger_level_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t size)
{
	return dbd2_reg_store(dev, DBD2_TG_THERSHOLD, attr, buf, size);
}

static ssize_t
dbd2_verification_level_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return dbd2_reg_show(dev, DBD2_VERIFICATION_THRESHOLD, attr, buf);
}

static ssize_t
dbd2_verification_level_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t size)
{
	return dbd2_reg_store(dev,DBD2_VERIFICATION_THRESHOLD,attr, buf,size );
}

static ssize_t
dbd2_gain_shift_factor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return dbd2_reg_show(dev,DBD2_GAIN_SHIFT_VALUE,attr, buf );
}

static ssize_t
dbd2_gain_shift_factor_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	return dbd2_reg_store(dev,DBD2_GAIN_SHIFT_VALUE,attr, buf,size);
}

static ssize_t
dbd2_io_addr_show(struct device *dev, struct device_attribute *attr,
		  char *buf)
{
	return dbd2_reg_show_long(dev, DBD2_IO_PORT_ADDR_HI,
				  DBD2_IO_PORT_ADDR_LO, attr, buf);
}

static ssize_t
dbd2_io_addr_store(struct device *dev, struct device_attribute *attr,
		   const char *buf, size_t size)
{
	return dbd2_reg_store_long(dev, DBD2_IO_PORT_ADDR_HI,
				   DBD2_IO_PORT_ADDR_LO, attr, buf, size);
}

static int
dbd2_load_new_acoustic_model(struct dbd2_data *dbd2)
{
	int ret;

	if (!dbd2->device_ready) {
		dev_err(dbd2->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	uart_lock(&dbd2->lock);
	ret = dbd2_wake(dbd2);
	uart_unlock(&dbd2->lock);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to wake\n");
		return ret;
	}
/*
	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_HOTPLUG,
				      dbd2->amodel_fw_name,
				      dbd2->dev,
				      GFP_KERNEL,
				      dbd2,
				      dbd2_acoustic_model_ready);
	if (ret < 0) {
		dev_err(dbd2->dev, "request_firmware_nowait error-%x\n",ret);
		return ret;
	}
*/

	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_HOTPLUG,
				      DBD2_GRAM_NAME,
				      gram_dev,
				      GFP_KERNEL,
				      dbd2,
				      dbd2_gram_bin_ready);
	if (ret < 0) {
		dev_err(dbd2->dev, "request_firmware_nowait error(%d)\n",ret);
		return ret;
	}
/*
	ret = request_firmware((const struct firmware **)&dbd2->dspg_gram,
				DBD2_GRAM_NAME, dbd2->dev);

	dbd2_gram_bin_ready(dbd2->dspg_gram, dbd2->dev);
*/
	dev_err(dbd2->dev, "gram firmware requested\n");

	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_HOTPLUG,
				      DBD2_NET_NAME,
				      net_dev,
				      GFP_KERNEL,
				      dbd2,
				      dbd2_net_bin_ready);
	if (ret < 0) {
		dev_err(dbd2->dev, "request_firmware_nowait error(%d)\n",ret);
		return ret;
	}
	dev_err(dbd2->dev, "net firmware requested\n");

	return 0;
}

static ssize_t
dbd2_acustic_model_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t size)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	int ret;
	int val = dbd2_buf_to_int(buf);

	if (val == 0) {
		mutex_lock(&dbd2->lock);
		dbd2->change_speed = 1;
		mutex_unlock(&dbd2->lock);
		/* 0 means load model */
		ret = dbd2_load_new_acoustic_model(dbd2);
	} else if (val == 1) {
		/* 1 means send 1 to register 0xF */
		uart_lock(&dbd2->lock);
		ret = dbd2_send_cmd(dbd2, DBD2_LOAD_NEW_ACUSTIC_MODEL | (u32)val, NULL);
		uart_unlock(&dbd2->lock);
		if (ret < 0) {
			dev_err(dbd2->dev, "failed to set DBD2_LOAD_NEW_ACUSTIC_MODEL to %d\n", val);
		} else {
			ret = 0;
		}
	} else {
		/* don't know what to do */
		ret = 0;
	}
	
	return (ret < 0? 0: size);
}

static ssize_t
dbd2_last_detect_verication_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return dbd2_reg_show(dev, DBD2_DETECT_VERIFICATION_LEVEL, attr, buf);
}

static ssize_t
dbd2_detect_trigger_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return dbd2_reg_show(dev, DBD2_DETECT_TRIGER_LEVEL, attr, buf);
}

static ssize_t
dbd2_last_detect_show(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	return dbd2_reg_show(dev, DBD2_LAST_DETECT_WORD_NUM, attr, buf);
}

static ssize_t
dbd2_max_sample_show(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	return dbd2_reg_show(dev, DBD2_LAST_MAX_SMP_VALUE, attr, buf);
}

static ssize_t
dbd2_io_value_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return dbd2_reg_show_long(dev, DBD2_IO_PORT_VALUE_HI,
				  DBD2_IO_PORT_VALUE_LO, attr, buf);
}

static ssize_t
dbd2_io_value_store(struct device *dev, struct device_attribute *attr,
		    const char *buf, size_t size)
{
	return dbd2_reg_store_long(dev, DBD2_IO_PORT_VALUE_HI,
				   DBD2_IO_PORT_VALUE_LO,attr, buf, size);
}

static ssize_t
dbd2_buffer_size_show(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	return dbd2_reg_show(dev, DBD2_AUDIO_BUFFER_SIZE, attr, buf);
}

static ssize_t
dbd2_buffer_size_store(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t size)
{
	return dbd2_reg_store(dev, DBD2_AUDIO_BUFFER_SIZE, attr, buf, size);
}

static ssize_t
dbd2_buffsmps_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return dbd2_reg_show(dev, DBD2_NUM_OF_SMP_IN_BUF, attr, buf);
}

static ssize_t
dbd2_audio_conv_show(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	return dbd2_reg_show(dev, DBD2_AUDIO_BUFFER_CONVERSION, attr, buf);
}

static ssize_t
dbd2_audio_conv_store(struct device *dev, struct device_attribute *attr,
		      const char *buf, size_t size)
{
	return dbd2_reg_store(dev, DBD2_AUDIO_BUFFER_CONVERSION, attr, buf,
			      size);
}

static ssize_t
dbd2_uartspeed_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	u16 reg;
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);

	uart_lock(&dbd2->lock);
	ret = dbd2_send_cmd_short(dbd2, DBD2_UART_SPEED, &reg);
	uart_unlock(&dbd2->lock);
	
	if (ret < 0)
		return -EIO;

	if (reg >= DBD2_UART_SPEEDS)
		return sprintf(buf, "unknown value %u\n", reg);

	return sprintf(buf, "%s\n", uart_speed_text[reg]);
}

static ssize_t
dbd2_uartspeed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	int val = dbd2_buf_to_int(buf);
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);

	if (val < 0)
		return -EIO;

	uart_unlock(&dbd2->lock);
	ret = dbd2_set_uart_speed(dbd2, val);
	uart_unlock(&dbd2->lock);

	if (ret != 0)
		return ret;

	return size;
}

static ssize_t
dbd2_lastduration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return dbd2_reg_show(dev,DBD2_LAST_DURATION,attr, buf );
}

static ssize_t
dbd2_lasterror_show(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	return dbd2_reg_show(dev, DBD2_LAST_ERROR, attr, buf);
}

static ssize_t
dbd2_micgain_show(struct device *dev, struct device_attribute *attr,
		  char *buf)
{
	return dbd2_reg_show(dev, DBD2_MIC_GAIN, attr, buf);
}

static ssize_t
dbd2_micgain_store(struct device *dev, struct device_attribute *attr,
		   const char *buf, size_t size)
{
	return dbd2_reg_store(dev, DBD2_MIC_GAIN, attr, buf, size);
}

static ssize_t
dbd2_backlog_size_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	return dbd2_reg_show(dev, DBD2_BUFFERING_BACKLOG_SIZE, attr, buf);
}

static ssize_t
dbd2_backlog_size_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t size)
{
	return dbd2_reg_store(dev, DBD2_BUFFERING_BACKLOG_SIZE, attr, buf,size );
}

static ssize_t
dbd2_detection_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	int ret;

	ret = sprintf(buf, "0x%x\n", dbd2->detection_state);

	return ret;
}

static ssize_t
dbd2_detection_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);

	dbd2->detection_state = 0;

	return size;
}

static ssize_t
dbd2_reset_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t size)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	struct dbd2_platform_data *pdata = &dbd2->pdata;
	int ret;
	int retry = 0;

	if (dbd2->client)
		return -EIO;

	dbd2->buffering = 0;

	flush_work(&dbd2->sensory_work);

	uart_lock(&dbd2->lock);

	dbd2_set_mode(dbd2, DBMD2_IDLE);

	dbd2->device_ready = false;

	dbd2_flush_rx_fifo(dbd2);

	/* set baudrate to FW baud (common case) */
	dbmd2_uart_configure_tty(dbd2->uart_tty,
				 UART_TTY_BOOT_BAUD_RATE,
				 UART_TTY_BOOT_STOP_BITS,
				 1, 0);

	do {
		/* Reset DBD2 chip */
		gpio_set_value(pdata->gpio_reset, 0);
		usleep_range(200, 300);
		gpio_set_value(pdata->gpio_reset, 1);

		/* Delay before sending commands */
		usleep_range(2000, 3000);

		dbd2_flush_rx_fifo(dbd2);

		ret = dbmd2_uart_sync(dbd2);

		dbd2_flush_rx_fifo(dbd2);

		if (!ret)
			break;

		retry++;
	} while (retry < 5);

	if (retry == 5) {
		dev_err(dbd2->dev, "multiple sync errors\n");
		
		uart_unlock(&dbd2->lock);
		return -EIO;
	}

	ret = dbd2_boot_firmware(dbd2);
	if (ret) {
		dev_err(dbd2->dev, "booting the firmware failed\n");
		
		uart_unlock(&dbd2->lock);
		return -EIO;
	}

	/* FIXME: wait some time till the bytes went out */
	msleep(10);

	/* set baudrate to FW baud (common case) */
	dbmd2_uart_configure_tty(dbd2->uart_tty,
					 UART_TTY_BAUD_RATE,
					 UART_TTY_STOP_BITS,
					 0, 1);
	dbd2_flush_rx_fifo(dbd2);

	ret = dbd2_wait_till_alive(dbd2);
	if (!ret) {
		dev_err(dbd2->dev, "failed to boot, device not responding\n");
		uart_unlock(&dbd2->lock);
		return -EIO;
	}

	if (dbd2->a_model_loaded) {
		ret = dbd2_send_cmd(dbd2, DBD2_LOAD_NEW_ACUSTIC_MODEL | 0x1, NULL);
		if (ret < 0) {
			dev_err(dbd2->dev, "failed to set DBD2_LOAD_NEW_ACUSTIC_MODEL\n");
			ret = -EIO;
			goto out;
		}
	}

out:
	dbd2->device_ready = true;
	uart_unlock(&dbd2->lock);

	return size;
}

static ssize_t
dbd2_readsmps_start(struct device *dev, struct device_attribute *attr,
		    const char *buf, size_t size)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	int ret;

	if (!dbd2->device_ready)
		return -EAGAIN;

	uart_lock(&dbd2->lock);
	ret = dbd2_wake(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to wake\n");
		goto out;
	}

	dbd2->buffering = 1;
	schedule_work(&dbd2->sensory_work);

out:
	uart_unlock(&dbd2->lock);
	return size ;
}


static ssize_t
dbd2_readsmps_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	int ret;
	u16 val = 0;
	int i = 0;

	if (!dbd2->device_ready)
		return -EAGAIN;

	uart_lock(&dbd2->lock);
	ret = dbd2_wake(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to wake\n");
		uart_unlock(&dbd2->lock);
		return ret;
	}

	/* A host command received will blocked until the current audio frame
	   processing is finished, which can take up to 10 ms */
	usleep_range(10000, 11000);


	val = 4000;

	#define SIZE 8
	for (i = 0; i <= val; i += SIZE) {
		char local_buf[SIZE];

		if (!dbd2->client) {
			if (dbmd2_uart_read(dbd2, local_buf, SIZE) < 0) {
				dev_err(dbd2->dev, "dbmd2_uart_read failed\n");
				
				uart_unlock(&dbd2->lock);
				return -EIO;
			}
		} else {
			ret = i2c_master_recv(dbd2->client,local_buf, SIZE);
			if (ret < 0) {
				dev_err(dbd2->dev, "i2c_master_recv failed\n");
				
				uart_unlock(&dbd2->lock);
				return ret;
			}
		}
		memcpy(buf + i, local_buf, SIZE);
	}
	#undef SIZE

	
	uart_unlock(&dbd2->lock);
	return val;
}

static ssize_t
dbd2_d2param_addr_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	if (!dbd2->device_ready)
		return -EAGAIN;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	uart_lock(&dbd2->lock);
	ret = dbd2_wake(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to wake\n");
		uart_unlock(&dbd2->lock);
		return ret;
	}

	ret = dbd2_send_cmd(dbd2, DBD2_SET_D2PARAM_ADDR | (u32)val, NULL);
	
	if (ret < 0) {
		dev_err(dbd2->dev, "set d2paramaddr error\n");
		uart_unlock(&dbd2->lock);
		return ret;
	}

	uart_unlock(&dbd2->lock);
	return size;
}

static ssize_t
dbd2_d2param_show(struct device *dev, struct device_attribute *attr,
		  char *buf)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	int ret;
	u16 val;

	if (!dbd2->device_ready)
		return -EAGAIN;

	uart_lock(&dbd2->lock);
	ret = dbd2_wake(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to wake\n");
		uart_unlock(&dbd2->lock);
		return ret;
	}

	ret = dbd2_send_cmd(dbd2, DBD2_GET_D2PARAM, &val);
	
	if (ret < 0) {
		dev_err(dbd2->dev, "get d2param error\n");
		uart_unlock(&dbd2->lock);
		return ret;
	}

	uart_unlock(&dbd2->lock);
	return sprintf(buf, "%u\n", val);
}

static ssize_t
dbd2_d2param_store(struct device *dev, struct device_attribute *attr,
		   const char *buf, size_t size)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	if (!dbd2->device_ready)
		return -EAGAIN;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	uart_lock(&dbd2->lock);
	ret = dbd2_wake(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to wake\n");
		uart_unlock(&dbd2->lock);
		return ret;
	}

	ret = dbd2_send_cmd(dbd2, DBD2_SET_D2PARAM | (u32)val, NULL);
	

	if (ret < 0) {
		dev_err(dbd2->dev, "set d2param error\n");
		uart_unlock(&dbd2->lock);
		return ret;
	}

	uart_unlock(&dbd2->lock);
	return size;
}

static ssize_t
dbd2_acoustic_model_filename_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	char *c;
	size_t len;

	mutex_lock(&dbd2->lock);
	kfree(dbd2->amodel_fw_name);
	dbd2->amodel_fw_name = kstrndup(buf, 100, GFP_KERNEL);
	len = strlen(dbd2->amodel_fw_name);
	if (len > 0) {
		c = &dbd2->amodel_fw_name[len - 1];
		if (*c == '\n')
			*c = '\0';
	}
	mutex_unlock(&dbd2->lock);
	return (size > 100? 100: size);
}

static ssize_t
dbd2_acoustic_model_filename_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&dbd2->lock);
	ret = sprintf(buf, "%s", dbd2->amodel_fw_name);
	mutex_unlock(&dbd2->lock);

	return ret;
}


static DEVICE_ATTR(fwver, S_IRUGO,
		   dbd2_fw_ver_show, NULL);
static DEVICE_ATTR(opmode,  S_IRUGO | S_IWUGO | S_IRUGO | S_IWUSR ,
		   dbd2_opr_mode_show, dbd2_opr_mode_store);
static DEVICE_ATTR(trigger, S_IRUGO | S_IWUSR,
		   dbd2_trigger_level_show, dbd2_trigger_level_store);
static DEVICE_ATTR(uart, S_IWUSR,
		   NULL, dbd2_uart_store);
static DEVICE_ATTR(verif, S_IRUGO | S_IWUSR,
		   dbd2_verification_level_show, dbd2_verification_level_store);
static DEVICE_ATTR(gain, S_IRUGO | S_IWUSR,
		   dbd2_gain_shift_factor_show, dbd2_gain_shift_factor_store);
static DEVICE_ATTR(io_addr, S_IRUGO | S_IWUSR,
		   dbd2_io_addr_show, dbd2_io_addr_store);
static DEVICE_ATTR(io_value, S_IRUGO | S_IWUSR,
		   dbd2_io_value_show, dbd2_io_value_store);
static DEVICE_ATTR(buffsize, S_IRUGO | S_IWUSR,
		   dbd2_buffer_size_show, dbd2_buffer_size_store);
static DEVICE_ATTR(buffsmps, S_IRUGO,
		   dbd2_buffsmps_show, NULL);
static DEVICE_ATTR(max_sample, S_IRUGO,
		   dbd2_max_sample_show, NULL);
static DEVICE_ATTR(detect_word, S_IRUGO,
		   dbd2_last_detect_show, NULL);
static DEVICE_ATTR(detect_trigger, S_IRUGO,
		   dbd2_detect_trigger_show, NULL);
static DEVICE_ATTR(detect_verification, S_IRUGO,
		   dbd2_last_detect_verication_show, NULL);
static DEVICE_ATTR(load_model, S_IWUGO | S_IWUSR ,
		   NULL,dbd2_acustic_model_store);
static DEVICE_ATTR(audioconv, S_IRUGO | S_IWUSR,
		   dbd2_audio_conv_show, dbd2_audio_conv_store);
static DEVICE_ATTR(readbuf, S_IRUGO | S_IWUSR,
		   dbd2_readsmps_show, dbd2_readsmps_start);
static DEVICE_ATTR(uartspeed, S_IRUGO | S_IWUGO,
		   dbd2_uartspeed_show, dbd2_uartspeed_store);
static DEVICE_ATTR(lastduration, S_IRUGO,
		   dbd2_lastduration_show, NULL);
static DEVICE_ATTR(lasterror, S_IRUGO,
		   dbd2_lasterror_show, NULL);
static DEVICE_ATTR(micgain, S_IRUGO | S_IWUSR,
		   dbd2_micgain_show, dbd2_micgain_store);
static DEVICE_ATTR(reset, S_IWUSR, NULL, dbd2_reset_store);
static DEVICE_ATTR(d2paramaddr, S_IWUSR,
		   NULL, dbd2_d2param_addr_store);
static DEVICE_ATTR(d2param, S_IRUGO | S_IWUSR,
		   dbd2_d2param_show, dbd2_d2param_store);
static DEVICE_ATTR(amodel_filename, S_IRUGO | S_IWUGO | S_IWUSR | S_IRUSR,
		   dbd2_acoustic_model_filename_show, dbd2_acoustic_model_filename_store);
static DEVICE_ATTR(backlog_size, S_IRUGO | S_IWUGO,
		   dbd2_backlog_size_show, dbd2_backlog_size_store);
static DEVICE_ATTR(detection, S_IRUGO | S_IWUGO,
		   dbd2_detection_show, dbd2_detection_store);

static struct attribute *dbd2_attributes[] = {
	&dev_attr_fwver.attr,
	&dev_attr_opmode.attr,
	&dev_attr_trigger.attr,
	&dev_attr_uart.attr,
	&dev_attr_verif.attr,
	&dev_attr_gain.attr,
	&dev_attr_io_addr.attr,
	&dev_attr_io_value.attr,
	&dev_attr_buffsize.attr,
	&dev_attr_buffsmps.attr,
	&dev_attr_max_sample.attr,
	&dev_attr_detect_word.attr,
	&dev_attr_detect_trigger.attr,
	&dev_attr_detect_verification.attr,
	&dev_attr_uartspeed.attr,
	&dev_attr_lastduration.attr,
	&dev_attr_lasterror.attr,
	&dev_attr_micgain.attr,
	&dev_attr_load_model.attr,
	&dev_attr_readbuf.attr,
	&dev_attr_audioconv.attr,
	&dev_attr_reset.attr,
	&dev_attr_d2paramaddr.attr,
	&dev_attr_d2param.attr,
	&dev_attr_amodel_filename.attr,
	&dev_attr_backlog_size.attr,
	&dev_attr_detection.attr,
	NULL,
};

static const struct attribute_group dbd2_attribute_group = {
	.attrs = dbd2_attributes,
};

/*
 * This is the callback function passed to request_firmware_nowait(),
 * and will be called as soon as the firmware is ready.
 */
static void
dbd2_firmware_ready(const struct firmware *fw, void *context)
{
	struct dbd2_data *dbd2 = (struct dbd2_data *)context;
	int ret;
	u16 fwver = 0xffff;

	if (!fw) {
		dev_err(dbd2->dev, "firmware request failed\n");
		return;
	}

	dev_info(dbd2->dev, "loading firmware\n");

	uart_lock(&dbd2->lock);

	dbd2->fw = fw;

	/* enable high speed clock for firmware loading */
//	dbd2_clk_enable(1);

	ret = dbd2_reset(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "unable to reset device\n");
		goto out_clk_off;
	}

	ret = dbd2_wait_till_alive(dbd2);
	if (!ret) {
		dev_err(dbd2->dev, "failed to boot, device not responding\n");
		goto out_clk_off;
	}

	/* disable high speed clock after firmware loading */
//	dbd2_clk_enable(0);

	(void)dbd2_send_cmd(dbd2, DBD2_AUDIO_BUFFER_SIZE | dbd2->audio_buffer_size/8, NULL);
	dbd2_set_bytes_per_sample(dbd2, dbd2->audio_mode);

	dbd2->device_ready = true;

	ret = dbd2_send_cmd_short(dbd2, DBD2_GET_FW_VER, &fwver);
	if (ret < 0)
		dev_err(dbd2->dev, "could not read firmware version\n");

	dev_info(dbd2->dev, "firmware 0x%x ready\n", fwver);

	goto out;

out_clk_off:
//	dbd2_clk_enable(0);
out:
	release_firmware(dbd2->fw);
	dbd2->fw = NULL;
	
	uart_unlock(&dbd2->lock);
}

int
dbmd2_get_samples(char *buffer, unsigned int samples)
{
	struct dbd2_data *dbd2 = dbd2_data;
	int avail = kfifo_len(&dbd2->pcm_kfifo);
	int samples_avail = avail / dbd2->bytes_per_sample;
	int ret;

	if (samples_avail < samples)
		return -1;

	ret = kfifo_out(&dbd2->pcm_kfifo,
			buffer,
			samples * dbd2->bytes_per_sample);

	return (ret == samples * dbd2->bytes_per_sample? 0: -1);
}
EXPORT_SYMBOL(dbmd2_get_samples);

int
dbmd2_codec_lock(void)
{
	if (!atomic_add_unless(&dbd2_data->audio_owner, 1, 1))
		return -EBUSY;

	return 0;
}
EXPORT_SYMBOL(dbmd2_codec_lock);

int
dbmd2_codec_unlock(void)
{
	atomic_dec(&dbd2_data->audio_owner);
	return 0;
}
EXPORT_SYMBOL(dbmd2_codec_unlock);

void
dbmd2_start_buffering(void)
{
	int ret;

	if (!dbd2_data->auto_buffering)
		return;

	uart_lock(&dbd2_data->lock);
	dbd2_data->buffering = 1;
	ret = dbd2_set_mode(dbd2_data, DBMD2_BUFFERING);
	uart_unlock(&dbd2_data->lock);
}
EXPORT_SYMBOL(dbmd2_start_buffering);

void
dbmd2_stop_buffering(void)
{
	int ret;

	if (!dbd2_data->auto_buffering)
		return;

	dbd2_data->buffering = 0;

	flush_work(&dbd2_data->sensory_work);

	uart_lock(&dbd2_data->lock);
	ret = dbd2_set_mode(dbd2_data, DBMD2_IDLE);
	uart_unlock(&dbd2_data->lock);
}
EXPORT_SYMBOL(dbmd2_stop_buffering);


/* ------------------------------------------------------------------------
 * codec driver section
 * ------------------------------------------------------------------------ */

#define DUMMY_REGISTER 0

static int
dbmd2_dai_hw_params(struct snd_pcm_substream *substream,
		  struct snd_pcm_hw_params *params,
		  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret = 0;
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(rtd->codec);

	mutex_lock(&dbd2->lock);
	if (dbd2_sleeping(dbd2)) {
		ret = -EIO;
		goto out_unlock;
	}
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dbd2_set_bytes_per_sample(dbd2, DBD2_AUDIO_MODE_PCM);
		break;
	case SNDRV_PCM_FORMAT_MU_LAW:
		dbd2_set_bytes_per_sample(dbd2, DBD2_AUDIO_MODE_MU_LAW);
		break;
	default:
		ret = -EINVAL;
	}

out_unlock:
	
	mutex_unlock(&dbd2->lock);

	return ret;
}

static struct snd_soc_dai_ops dbmd2_dai_ops = {
	.hw_params = dbmd2_dai_hw_params,
};

/* DBMD2 codec DAI: */
struct snd_soc_dai_driver dbmd2_dais[] = {
	{
		.name = "DBMD2_codec_dai",
		.capture = {
			.stream_name	= "vs_buffer",
			.channels_min	= 1,
			.channels_max	= 1,
			.rates		= SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats	= SNDRV_PCM_FMTBIT_MU_LAW | SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &dbmd2_dai_ops,
	},
};

/* ASoC controls */
static unsigned int
dbmd2_dev_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(codec);
	int ret;
	u16 val = 0;
	dbd2 = dbd2_data;

	if (reg == DUMMY_REGISTER)
		return 0;

	uart_lock(&dbd2->lock);


	/* TODO maybe just return and the user needs to wakeup */
	if (dbd2_sleeping(dbd2)) {
		dev_err(dbd2->dev, "device sleeping\n");
		goto out_unlock;
	}

	ret = dbd2_send_cmd_short(dbd2, 0x80000000 | ((reg & 0xff) << 16), &val);
	if (ret < 0)
		dev_err(dbd2->dev, "read 0x%x error\n", reg);

out_unlock:
	
	uart_unlock(&dbd2->lock);
	return (unsigned int)val;
}

static int
dbmd2_dev_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int val)
{
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(codec);
	int ret;
	dbd2 = dbd2_data;

	if (!snd_soc_codec_writable_register(codec, reg)) {
		printk("register not writable\n");
		return -EIO;
	}

	if (reg == DUMMY_REGISTER)
		return 0;

	uart_lock(&dbd2->lock);


	/* TODO maybe just return and the user needs to wakeup */
	if (dbd2_sleeping(dbd2)) {
		ret = -EIO;
		dev_err(dbd2->dev, "device sleeping\n");
		goto out_unlock;
	}

	ret = dbd2_send_cmd(dbd2, 0x80000000 | ((reg & 0xff) << 16) |
			    (val & 0xffff), NULL);
	if (ret < 0)
		dev_err(dbd2->dev, "write 0x%x to 0x%x error\n", val, reg);

out_unlock:
	

	uart_unlock(&dbd2->lock);

	return ret;
}

static int
dbmd2_control_get(struct snd_kcontrol *kcontrol,
		  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(codec);
	unsigned short val, reg = mc->reg;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;
	int ret;
	dbd2 = dbd2_data;

	uart_lock(&dbd2->lock);

	/* TODO maybe just return and the user needs to wakeup */
	if (dbd2_sleeping(dbd2)) {
		dev_err(dbd2->dev, "device sleeping\n");
		goto out_unlock;
	}

	ret = dbd2_send_cmd_short(dbd2, 0x80000000 | ((reg & 0xff) << 16), &val);
	if (ret < 0)
		dev_err(dbd2->dev, "read 0x%x error\n", reg);

	val &= mask;

	ucontrol->value.integer.value[0] = val;

out_unlock:
	
	uart_unlock(&dbd2->lock);

	return 0;
}

static int
dbmd2_control_put(struct snd_kcontrol *kcontrol,
		  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(codec);
	unsigned short val = ucontrol->value.integer.value[0];
	unsigned short reg = mc->reg;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;
	int ret;
	dbd2 = dbd2_data;

	if (!snd_soc_codec_writable_register(codec, reg)) {
		printk("register not writable\n");
		return -EIO;
	}

	val &= mask;

	uart_lock(&dbd2->lock);

	;

	/* TODO maybe just return and the user needs to wakeup */
	if (dbd2_sleeping(dbd2)) {
		dev_err(dbd2->dev, "device sleeping\n");
		goto out_unlock;
	}

	ret = dbd2_send_cmd(dbd2, 0x80000000 | ((reg & 0xff) << 16) |
			    (val & 0xffff), NULL);
	if (ret < 0)
		dev_err(dbd2->dev, "write 0x%x to 0x%x error\n", val, reg);

out_unlock:
	

	uart_unlock(&dbd2->lock);

	return 0;
}

/* Operation modes */
static int
dbmd2_operation_mode_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(codec);
	unsigned short val;
	int ret;

	dbd2 = dbd2_data;
	if (dbd2_sleeping(dbd2)) {
		/* report hibernate */
		ucontrol->value.integer.value[0] = 2;
		return 0;
	}

	uart_lock(&dbd2->lock);

	

	ret = dbd2_send_cmd_short(dbd2, DBD2_OPR_MODE, &val);
	if (ret < 0) {
		dev_err(dbd2->dev, "failed to read DBD2_OPR_MODE\n");
		goto out_unlock;
	}

	if (val == DBMD2_SLEEP)
		ucontrol->value.integer.value[0] = 1;
	else if (val == DBMD2_HIBERNATE)
		ucontrol->value.integer.value[0] = 2;
	else if (val == DBMD2_BUFFERING)
		ucontrol->value.integer.value[0] = 4;
	else if (val == DBMD2_DETECTION)
		ucontrol->value.integer.value[0] = 3;
	else if (val == DBMD2_IDLE)
		ucontrol->value.integer.value[0] = 0;
	else
		dev_err(dbd2->dev, "unknown operation mode: %u\n", val);

out_unlock:
	

	uart_unlock(&dbd2->lock);

	return ret;
}

static int
dbmd2_operation_mode_set(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	dbd2 = dbd2_data;

	uart_lock(&dbd2->lock);

	if (ucontrol->value.integer.value[0] == 0)
		dbd2_set_mode(dbd2, DBMD2_IDLE);
	else if (ucontrol->value.integer.value[0] == 1)
		dbd2_set_mode(dbd2, DBMD2_SLEEP);
	else if (ucontrol->value.integer.value[0] == 2)
		dbd2_set_mode(dbd2, DBMD2_HIBERNATE);
	else if (ucontrol->value.integer.value[0] == 3)
		dbd2_set_mode(dbd2, DBMD2_DETECTION);
	else if (ucontrol->value.integer.value[0] == 4)
		dbd2_set_mode(dbd2, DBMD2_BUFFERING);
	else
		ret = -EINVAL;

	
	uart_unlock(&dbd2->lock);

	return ret;
}

static const char *dbmd2_operation_mode_texts[] = {
	"Idle", "Sleep", "Hibernate", "Detection", "Buffering",
};

static const struct soc_enum dbmd2_operation_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dbmd2_operation_mode_texts),
			    dbmd2_operation_mode_texts);

static DECLARE_TLV_DB_SCALE(dbmd2_db_tlv, -3276700, 3276800, 0);

static const unsigned int dbmd2_mic_analog_gain_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 21, TLV_DB_SCALE_ITEM(-400, 200, 0),
};

static int
dbmd2_amodel_load_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int
dbmd2_amodel_load_set(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(codec);
	dbd2 = dbd2_data;

	mutex_lock(&dbd2->lock);
	dbd2->change_speed = 1;
	mutex_unlock(&dbd2->lock);

	/* trigger loading of new acoustic model */
	return dbd2_load_new_acoustic_model(dbd2);
}

static int
dbmd2_wakeup_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(codec);
	dbd2 = dbd2_data;
	ucontrol->value.enumerated.item[0] = dbd2_sleeping(dbd2);

	return 0;
}

static int
dbmd2_wakeup_set(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(codec);
	//struct dbd2_platform_data *pdata = &dbd2->pdata;
	int value = ucontrol->value.enumerated.item[0];
	dbd2 = dbd2_data;

	mutex_lock(&dbd2->lock);
	if (value) {
		/* TODO */
		//gpio_set_value(pdata->gpio_wakeup, 1);
		dbd2->asleep = true;
	} else {
		/* TODO */
		//gpio_set_value(pdata->gpio_wakeup, 0);
		dbd2->asleep = false;
	}
	mutex_unlock(&dbd2->lock);
	return 0;
}

static const struct snd_kcontrol_new dbmd2_snd_controls[] = {
	SOC_ENUM_EXT("Operation mode", dbmd2_operation_mode_enum,
		dbmd2_operation_mode_get, dbmd2_operation_mode_set),
	SOC_SINGLE_EXT_TLV("Trigger threshold", 0x02, 0, 0xffff, 0,
		dbmd2_control_get, dbmd2_control_put, dbmd2_db_tlv),
	SOC_SINGLE_EXT("Verification threshold", 0x03, 0, 8192, 0,
		dbmd2_control_get, dbmd2_control_put),
	SOC_SINGLE_EXT("Gain shift value", 0x04, 0, 15, 0,
		dbmd2_control_get, dbmd2_control_put),
	SOC_SINGLE_EXT_TLV("Microphone analog gain", 0x16, 0, 0xffff, 0,
		dbmd2_control_get, dbmd2_control_put,
		dbmd2_mic_analog_gain_tlv),
	SOC_SINGLE_BOOL_EXT("Load acoustic model",
			    0,
			    dbmd2_amodel_load_get,
			    dbmd2_amodel_load_set),
	SOC_SINGLE_BOOL_EXT("Set wakeup",
			    0,
			    dbmd2_wakeup_get,
			    dbmd2_wakeup_set),
	SOC_SINGLE("Word ID", 0x0c, 0, 0xffff, 0),
	SOC_SINGLE("Trigger Level", 0x0d, 0, 0xffff, 0),
	SOC_SINGLE("Verification Level", 0x0e, 0, 0xffff, 0),
	SOC_SINGLE("Duration", 0x14, 0, 0xffff, 0),
	SOC_SINGLE("Error", 0x15, 0, 0xffff, 0),
	SOC_SINGLE_EXT("Backlog size", 0x1B, 0, 0xffff, 0,
		dbmd2_control_get, dbmd2_control_put),
/*

#define SOC_SINGLE_EXT(xname, xreg, xshift, xmax, xinvert,
	 xhandler_get, xhandler_put)
*/
};

static int
dbmd2_set_bias_level(struct snd_soc_codec *codec,
		   enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		break;
	default:
		return -EINVAL;
	}

	/* change to new state */
	codec->dapm.bias_level = level;

	return 0;
}

static int
dbmd2_dev_probe(struct snd_soc_codec *codec)
{
	codec->control_data = NULL;

	return 0;
}

static int
dbmd2_dev_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static int
dbmd2_dev_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int
dbmd2_dev_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static int
dbmd2_is_writeable_register(struct snd_soc_codec *codec, unsigned int reg)
{
	struct dbd2_data *dbd2 = snd_soc_codec_get_drvdata(codec);
	int ret = 1;
	dbd2 = dbd2_data;

	switch (reg) {
	case 0xc:
	case 0xd:
	case 0xe:
	case 0x14:
	case 0x15:
		ret = 0;
		break;
	default:
		break;
	}
	return ret;
}

static struct snd_soc_codec_driver soc_codec_dev_dbmd2 = {
	.probe   = dbmd2_dev_probe,
	.remove  = dbmd2_dev_remove,
	.suspend = dbmd2_dev_suspend,
	.resume  = dbmd2_dev_resume,
	.set_bias_level = dbmd2_set_bias_level,
	.read = dbmd2_dev_read,
	.write = dbmd2_dev_write,
	.controls = dbmd2_snd_controls,
	.num_controls = ARRAY_SIZE(dbmd2_snd_controls),
	.writable_register = dbmd2_is_writeable_register,

	.reg_cache_size = 0,
	.reg_word_size = 0,
	.reg_cache_default = NULL,
	.ignore_pmdown_time = true,
};

int dbmd2_remote_add_codec_controls(struct snd_soc_codec *codec) {
	int rc;

	printk("%s start\n", __func__);

	rc = snd_soc_add_codec_controls(codec, dbmd2_snd_controls,
				ARRAY_SIZE(dbmd2_snd_controls));

	if(rc)
		dev_err(codec->dev, "%s(): dbmd2_remote_add_codec_controls failed\n", __func__);

	return rc;
}

static int
dbmd2_read_data(struct dbd2_data *dbd2, unsigned int bytes_to_read)
{
	unsigned int count;
	int ret = -EIO;
	unsigned int i;
	char tbuf[UART_TTY_READ_SZ];

	#define SIZE 8
	if (!dbd2->client) {
		count = UART_TTY_READ_SZ;
	} else {
		count = SIZE;
	}
	if (count > bytes_to_read)
		count = bytes_to_read;
	for (i = 0; i < bytes_to_read; i += count) {
		char local_buf[SIZE];
		if ((i + count) > bytes_to_read) {
			count = bytes_to_read - i;
		}
		if (!dbd2->client) {
			ret = dbmd2_uart_read_sync(dbd2, tbuf, count);
			if (ret < 0) {
				dev_err(dbd2->dev, "%s: dbmd2_uart_read failed %d\n", __func__, ret);
				dev_err(dbd2->dev, "%s: %d/%d/%d\n", __func__, i, bytes_to_read, count);
				dev_err(dbd2->dev, "%s: kfifo avil = %d\n", __func__, kfifo_avail(&dbd2->pcm_kfifo));
				goto out;
			}
			kfifo_in(&dbd2->pcm_kfifo, tbuf, count);
			//printk("dbd2: kfifo len %d\n", kfifo_len(&dbd2->pcm_kfifo));
		} else {
			if (i2c_master_recv(dbd2->client, local_buf, count) < 0) {
				dev_err(dbd2->dev, "i2c_master_recv failed\n");
				goto out;
			}
			kfifo_in(&dbd2->pcm_kfifo, local_buf, count);
			//smp_wmb();
		}
	}
	#undef SIZE
	ret = 0;
out:
	return ret;
}

static void
dbmd2_uevent_work(struct work_struct *work)
{
	struct dbd2_data *dbd2 = container_of(work, struct dbd2_data,
		uevent_work);
	char tmp[100];
	char *envp[] = { tmp, NULL };

	printk("%s\n", __func__);

	snprintf(tmp, sizeof(tmp), "VOICE_WAKEUP_WORD_ID=1");
	kobject_uevent_env(&dbd2->dev->kobj, KOBJ_CHANGE, envp);
}

static void
dbmd2_sensory_work(struct work_struct *work)
{
	struct dbd2_data *dbd2 = container_of(work, struct dbd2_data,
		sensory_work);
	int ret;
	int state;
	int bytes_per_sample = dbd2->bytes_per_sample;
	unsigned int bytes_to_read;
	u16 nr_samples;
	unsigned int total = 0;
	int kfifo_space = 0;

	printk("%s\n", __func__);

	if (dbd2_sleeping(dbd2)) {
		/* deassert wakeup */
		//gpio_set_value(pdata->gpio_wakeup, 0);
	}

	//mutex_lock(&dbd2->lock_data_transfer);
	uart_lock(&dbd2->lock);


	/* flush fifo */
	kfifo_reset(&dbd2->pcm_kfifo);

	if (!dbd2->client) {
		ret = dbd2_set_uart_speed(dbd2, DBD2_UART_SPEED_460800);
		if (ret) {
			dev_err(dbd2->dev, "failed switch to higher speed\n");
			goto out_fail_unlock;
		}
	}

	uart_unlock(&dbd2->lock);

	do {
		uart_lock(&dbd2->lock);
		bytes_to_read = 0;

		/* read number of samples available in audio buffer */
		if (dbd2_send_cmd_short(dbd2, DBD2_NUM_OF_SMP_IN_BUF, &nr_samples) < 0) {
			dev_err(dbd2->dev, "failed to read DBD2_NUM_OF_SMP_IN_BUF\n");
			uart_unlock(&dbd2->lock);
			goto out_fail;
		}

		/* check if we are done */
		state = dbd2_get_mode(dbd2);
		if (((state != DBMD2_BUFFERING) && (nr_samples ==0)) || !dbd2->buffering) {
			dev_err(dbd2->dev, "buffering mode left with %u samples, state %u\n", nr_samples, state);
			uart_unlock(&dbd2->lock);
			break;
		}

//		printk("%s: in the middle of get data(%d) state(%d)\n", __func__, nr_samples, state);

		uart_unlock(&dbd2->lock);

		/* Now fill the kfifo. The user can access the data in
		 * parallel. The kfifo is safe for concurrent access of one
		 * reader (ALSA-capture/character device) and one writer (this
		 * work-queue). */
		if (nr_samples) {

			
			bytes_to_read = nr_samples * 8 * bytes_per_sample;

			/* limit transcation size (no software flow control ) */
			if (bytes_to_read > UART_TTY_MAX_HW_BUF_SIZE)
				bytes_to_read = UART_TTY_MAX_HW_BUF_SIZE;
				
			kfifo_space = kfifo_avail(&dbd2->pcm_kfifo);

			if (bytes_to_read > kfifo_space) {
				bytes_to_read = kfifo_space;
			}

			nr_samples = bytes_to_read / (8 * bytes_per_sample);

			if(nr_samples) {
				uart_lock(&dbd2->lock);
//				printk("%s: nr_samples(%d)\n", __func__, nr_samples);
				/* activate reading of audio buffer with nr of samples to read */
				if (dbd2_send_cmd(dbd2, DBD2_POST_TRIGGER_AUDIO_BUF | nr_samples, NULL) < 0) {
					dev_err(dbd2->dev, "failed to write DBD2_POST_TRIGGER_AUDIO_BUF\n");
					uart_unlock(&dbd2->lock);
					goto out_fail;
				}

				dbmd2_read_data(dbd2, bytes_to_read);
				uart_unlock(&dbd2->lock);
			} else
				msleep(300);
		}
		else
			msleep(100);
		//uart_unlock(&dbd2->lock);

		total += bytes_to_read;

	} while (1);

	dbd2->audio_processed = 0;
	dev_info(dbd2->dev, "audio buffer read, total of %u bytes\n", total);

out_fail:
	if (!dbd2->client && dbd2->buffering) {
		/* wait till firmware has finished buffering */
		do {
			uart_lock(&dbd2->lock);
			state = dbd2_get_mode(dbd2);
			/* wait around 50ms if state not changed */
			uart_unlock(&dbd2->lock);
			if (state > DBMD2_IDLE)
				usleep_range(50000, 51000);
		} while (state == DBMD2_BUFFERING /*> DBMD2_IDLE*/);

		if (state < DBMD2_IDLE) {
			dev_err(dbd2->dev, "failed to wait till audio buffering finished\n");
//			goto out_fail_unlock;
		}
	}

	uart_lock(&dbd2->lock);

	if (!dbd2->client) {
		ret = dbd2_set_uart_speed(dbd2, DBD2_UART_SPEED_57600);
		if (ret) {
			dev_err(dbd2->dev, "failed switch to higher speed\n");
		}
	}
out_fail_unlock:
	uart_unlock(&dbd2->lock);
	//mutex_unlock(&dbd2->lock_data_transfer);
}

static irqreturn_t
dbmd2_sensory_interrupt(int irq, void *dev)
{
	struct dbd2_data *dbd2 = (struct dbd2_data *)dev;

	printk("%s\n", __func__);

	if (dbd2->device_ready) {
		dbd2->buffering = 1;
		dbd2->detection_state = 1;
		schedule_work(&dbd2->sensory_work);

		schedule_work(&dbd2->uevent_work);

		printk("SENSORY EVENT\n");
	}

/*
	input_report_key(dbd2->input, KEY_VOICE_WAKEUP, 1);
	input_sync(dbd2->input);
//	msleep(10);
	input_report_key(dbd2->input, KEY_VOICE_WAKEUP, 0);
	input_sync(dbd2->input);
*/

	//dbd2_set_mode(dbd2, DBMD2_DETECTION);

	return IRQ_HANDLED;
}

/* Access to the audio buffer is controlled through "audio_owner". Either the
 * character device or the ALSA-capture device can be opened. */
static int
dbmd2_record_open(struct inode *inode, struct file *file)
{
	printk("%s\n", __func__);

	file->private_data = dbd2_data;

	if (!atomic_add_unless(&dbd2_data->audio_owner, 1, 1))
		return -EBUSY;

	//dbd2_uart_clk_enable(1);

	return 0;
}

static int
dbmd2_record_release(struct inode *inode, struct file *file)
{
//	struct dbd2_data *dbd2 = (struct dbd2_data *)file->private_data;
//	int ret;

	printk("%s\n", __func__);

	uart_lock(&dbd2_data->lock);
	dbd2_data->buffering = 0;
	uart_unlock(&dbd2_data->lock);

	flush_work(&dbd2_data->sensory_work);

	atomic_dec(&dbd2_data->audio_owner);

	return 0;
}

/* The write function is a hack to load the A-model on systems where the
 * firmware files are not accesible to the user. */
static ssize_t
dbmd2_record_write(struct file *file, const char __user *buf, size_t count_want,
		  loff_t *f_pos)
{
	printk("%s\n", __func__);

	return count_want;
}

static ssize_t
dbmd2_record_read(struct file *file, char __user *buf, size_t count_want,
		  loff_t *f_pos)
{
	struct dbd2_data *dbd2 = (struct dbd2_data *)file->private_data;
	size_t not_copied;
	ssize_t to_copy = count_want;
	int avail;
	unsigned int copied, total_copied = 0;
	int ret;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	printk("%s: count_want = %d\n", __func__, count_want);

	while ((total_copied < count_want) && time_before(jiffies, timeout) && dbd2->buffering) {
		avail = kfifo_len(&dbd2->pcm_kfifo);

		if (avail == 0) {
			msleep(10);
			continue;
		}

		to_copy = avail;
		if (count_want - total_copied < avail)
			to_copy = count_want - total_copied;

		//mutex_lock(&dbd2->lock);

		ret = kfifo_to_user(&dbd2->pcm_kfifo, buf + total_copied, to_copy, &copied);
		if (ret) {
			//mutex_unlock(&dbd2->lock);
			return -EIO;
		}

		total_copied += copied;
		//mutex_unlock(&dbd2->lock);
	}
	if (total_copied < count_want) {
		printk("dbmd2: timeout during reading\n");
	}

	not_copied = count_want - total_copied;
	*f_pos = *f_pos + (count_want - not_copied);
	printk("%s: copied = %d\n", __func__, count_want-not_copied);

	return (count_want - not_copied);
}

static struct file_operations record_fops = {
	.owner   = THIS_MODULE,
	.open    = dbmd2_record_open,
	.release = dbmd2_record_release,
	.read    = dbmd2_record_read,
	.write   = dbmd2_record_write,
};

int dbmd2_init_input_device(struct dbd2_data *dbd2){
	int rc;
	dbd2->input = input_allocate_device();
	if(!dbd2->input){
		rc = -ENOMEM;
		goto dbmd2_init_input_device_exit;
	}

	dbd2->input->name = "dbd2 input";
	set_bit(EV_SYN, dbd2->input->evbit);
	set_bit(EV_KEY, dbd2->input->evbit);
	set_bit(KEY_VOICE_WAKEUP, dbd2->input->keybit);
	set_bit(KEY_VOICE_WAKEUP_LPSD, dbd2->input->keybit);

	rc = input_register_device(dbd2->input);
	if(rc < 0)
		input_free_device(dbd2->input);

dbmd2_init_input_device_exit:
	return rc;
}

static int
dbmd2_common_probe(struct dbd2_data *dbd2)
{
	struct device_node *np = dbd2->dev->of_node;
	struct dbd2_platform_data *pdata;
	int ret = 0;

	dbd2_data = dbd2;
	dev_set_drvdata(dbd2->dev, dbd2);
	pdata = &dbd2->pdata;

	dbd2->amodel_fw_name = kzalloc(strlen(DBD2_MODEL_NAME), GFP_KERNEL);
	if (!dbd2->amodel_fw_name) {
		dev_err(dbd2->dev, "out of memory\n");
		goto err_kfree;
	}
	strncpy(dbd2->amodel_fw_name, DBD2_MODEL_NAME, strlen(DBD2_MODEL_NAME));

	dbd2->amodel_buf = vmalloc(MAX_AMODEL_SIZE);
	if (!dbd2->amodel_buf) {
		dev_err(dbd2->dev, "out of memory\n");
		goto err_kfree2;
	}

	atomic_set(&dbd2->audio_owner, 0);

	if (!np) {
		dev_err(dbd2->dev, "error no devicetree entry\n");
		ret = -ENODEV;
		goto err_kfree3;
	}

/* reset */
	pdata->gpio_reset = of_get_named_gpio(np, "reset-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_reset)) {
		dev_err(dbd2->dev, "reset gpio invalid\n");
		ret = -EINVAL;
		goto err_kfree3;
	}

	ret = gpio_request(pdata->gpio_reset, "DBMD2 reset");
	if (ret < 0) {
		dev_err(dbd2->dev, "error requesting reset gpio\n");
		goto err_kfree3;
	}
	gpio_direction_output(pdata->gpio_reset, 0);
	gpio_set_value(pdata->gpio_reset, 0);

	dbd2_clk_enable(true);
	msleep(100);

/* sensory */
	pdata->gpio_sensory = of_get_named_gpio(np, "sensory-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_sensory)) {
		dev_err(dbd2->dev, "sensory gpio invalid %d\n",
			pdata->gpio_sensory);
		goto err_gpio_free;
	}

	ret = gpio_request(pdata->gpio_sensory, "DBMD2 sensory");
	if (ret < 0) {
		dev_err(dbd2->dev, "error requesting sensory gpio\n");
		goto err_gpio_free;
	}
	gpio_direction_input(pdata->gpio_sensory);

/* Add input device */
	ret = dbmd2_init_input_device(dbd2);

// Interrupt gpio
	pdata->sensory_irq = ret = gpio_to_irq(pdata->gpio_sensory);
	if (ret < 0) {
		dev_err(dbd2->dev, "cannot mapped gpio to irq\n");
		goto err_gpio_free2;
	}

/* wakeup */
	pdata->gpio_wakeup = of_get_named_gpio(np, "wakeup-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_wakeup)) {
		dev_err(dbd2->dev, "wakeup gpio invalid %d\n",
			pdata->gpio_wakeup);
		goto err_gpio_free2;
	}

	ret = gpio_request(pdata->gpio_wakeup, "DBD2 wakeup");
	if (ret < 0) {
		dev_err(dbd2->dev, "error requesting wakeup gpio\n");
		goto err_gpio_free2;
	}
	/* keep the wakeup pin low */
	gpio_direction_output(pdata->gpio_wakeup, 0);
	gpio_set_value(pdata->gpio_wakeup, 0);

	INIT_WORK(&dbd2->sensory_work, dbmd2_sensory_work);
	INIT_WORK(&dbd2->uevent_work, dbmd2_uevent_work);
	mutex_init(&dbd2->lock);
//	mutex_init(&dbd2->lock_data_transfer);
	ret = kfifo_alloc(&dbd2->pcm_kfifo, MAX_KFIFO_BUFFER_SIZE, GFP_KERNEL);
	if (ret) {
		dev_err(dbd2->dev, "no kfifo memory\n");
		goto err_gpio_free3;
	}

	dbd2->audio_buffer_size = MAX_AUDIO_BUFFER_SIZE;
	dbd2->audio_mode = DBD2_AUDIO_MODE_PCM;
	dbd2->audio_processed = 1;

	ret = of_property_read_u32(np, "buffer_size", &dbd2->audio_buffer_size);
	if ((ret && ret != -EINVAL) || (dbd2->audio_buffer_size > MAX_AUDIO_BUFFER_SIZE)) {
		dev_err(dbd2->dev, "invalid 'buffer_size'\n");
		goto err_kfifo_free;
	}
	/* round audio buffer size down to multiple of 240 */
	dbd2->audio_buffer_size = (dbd2->audio_buffer_size / 240) * 240;

	ret = of_property_read_u32(np, "audio_mode", &dbd2->audio_mode);
	if ((ret && ret != -EINVAL) ||
	    (dbd2->audio_mode != 0 && dbd2->audio_mode != 1)) {
		dev_err(dbd2->dev, "invalid 'audio_mode'\n");
		goto err_kfifo_free;
	}

	ret = of_property_read_u32(np, "auto_buffering", &dbd2->auto_buffering);
	if ((ret && ret != -EINVAL) ||
	    (dbd2->auto_buffering != 0 && dbd2->auto_buffering != 1)) {
		dev_err(dbd2->dev, "invalid 'auto_buffering'\n");
		goto err_kfifo_free;
	}

	dev_info(dbd2->dev, "request_firmware - %s\n",DBD2_FIRMWARE_NAME);
	request_firmware_nowait(THIS_MODULE,
				FW_ACTION_HOTPLUG,
				DBD2_FIRMWARE_NAME,
				dbd2->dev,
				GFP_KERNEL,
				dbd2,
				dbd2_firmware_ready);

	ret = request_irq(pdata->sensory_irq, dbmd2_sensory_interrupt,
			  IRQF_TRIGGER_RISING /* | IRQF_TRIGGER_FALLING */,
			  "dbmd2_sensory", dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "cannot get irq\n");
		goto err_kfifo_free;
	}

	ret = irq_set_irq_wake(pdata->sensory_irq, 1);

	if (ret < 0) {
		dev_err(dbd2->dev, "cannot set irq_set_irq_wake\n");
		goto err_free_irq;
	}

	ns_class = class_create(THIS_MODULE, "voice_trigger");
	if (IS_ERR(ns_class)) {
		dev_err(dbd2->dev, "failed to create class\n");
		goto err_free_irq;
	}

	dbd2_dev = device_create(ns_class, NULL, 0, dbd2, "dbd2");
	if (IS_ERR(dbd2_dev)) {
		dev_err(dbd2->dev, "could not create device\n");
		goto err_class_destroy;
	}

	gram_dev = device_create(ns_class, NULL, 0, dbd2, "gram");
	if (IS_ERR(gram_dev)) {
		dev_err(dbd2->dev, "could not create device\n");
		goto err_class_destroy;
	}

	net_dev = device_create(ns_class, NULL, 0, dbd2, "net");
	if (IS_ERR(net_dev)) {
		dev_err(dbd2->dev, "could not create device\n");
		goto err_class_destroy;
	}

	ret = sysfs_create_group(&dbd2_dev->kobj, &dbd2_attribute_group);
	if (ret) {
		dev_err(dbd2_dev, "failed to create sysfs group\n");
		goto err_device_unregister;
	}

	ret = alloc_chrdev_region(&dbd2->record_chrdev, 0, 1, "dbd2");
	if (ret) {
		dev_err(dbd2_dev, "failed to allocate character device\n");
		goto err_sysfs_remove_group;
	}

	cdev_init(&dbd2->record_cdev, &record_fops);

	dbd2->record_cdev.owner = THIS_MODULE;

	ret = cdev_add(&dbd2->record_cdev, dbd2->record_chrdev, 1);
	if (ret) {
		dev_err(dbd2_dev, "failed to add character device\n");
		goto err_unregister_chrdev_region;
	}

	dbd2->record_dev = device_create(ns_class, &platform_bus,
					 MKDEV(MAJOR(dbd2->record_chrdev), 0),
					 dbd2, "dbd2%d", 0);
	if (IS_ERR(dbd2->record_dev)) {
		dev_err(dbd2->dev, "could not create device\n");
		goto err_cdev_del;
	}

	/* register the codec */
	ret = snd_soc_register_codec(dbd2->dev,
				     &soc_codec_dev_dbmd2,
				     dbmd2_dais,
				     ARRAY_SIZE(dbmd2_dais));
	if (ret != 0) {
		dev_err(dbd2->dev,
			"Failed to register codec and its DAI: %d\n",  ret);
		goto err_device_unregister2;
	}

	

	dev_info(dbd2->dev, "registered DBMD2 codec driver\n");

	return 0;

err_device_unregister2:
	device_unregister(dbd2->record_dev);
err_cdev_del:
	cdev_del(&dbd2->record_cdev);
err_unregister_chrdev_region:
	unregister_chrdev_region(dbd2->record_chrdev, 1);
err_sysfs_remove_group:
	sysfs_remove_group(&dbd2->dev->kobj, &dbd2_attribute_group);
err_device_unregister:
	device_unregister(dbd2_dev);
err_class_destroy:
	class_destroy(ns_class);
err_kfifo_free:
	kfifo_free(&dbd2->pcm_kfifo);
err_free_irq:
	free_irq(pdata->sensory_irq, dbd2);
err_gpio_free3:
	gpio_free(dbd2->pdata.gpio_wakeup);
err_gpio_free2:
	gpio_free(dbd2->pdata.gpio_sensory);
err_gpio_free:
	gpio_free(dbd2->pdata.gpio_reset);
err_kfree3:
	kfree(dbd2->amodel_fw_name);
err_kfree2:
	vfree(dbd2->amodel_buf);
err_kfree:
	kfree(dbd2);

	return ret;
}

static int
dbmd2_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct dbd2_data *dbd2;

	dbd2 = kzalloc(sizeof(*dbd2), GFP_KERNEL);
	if (dbd2 == NULL) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	dbd2->client = client;
	dbd2->dev = &client->dev;

	return dbmd2_common_probe(dbd2);
}

static void
dbmd2_common_remove(struct dbd2_data *dbd2)
{
	snd_soc_unregister_codec(dbd2->dev);
	device_unregister(dbd2->record_dev);
	cdev_del(&dbd2->record_cdev);
	unregister_chrdev_region(dbd2->record_chrdev, 1);
	sysfs_remove_group(&dbd2->dev->kobj, &dbd2_attribute_group);
	device_unregister(dbd2_dev);
	class_destroy(ns_class);
	kfifo_free(&dbd2->pcm_kfifo);
	free_irq(dbd2->pdata.sensory_irq, dbd2);
	gpio_free(dbd2->pdata.gpio_wakeup);
	gpio_free(dbd2->pdata.gpio_reset);
	gpio_free(dbd2->pdata.gpio_sensory);
	kfifo_free(&dbd2->pcm_kfifo);
	kfree(dbd2->amodel_fw_name);
	vfree(dbd2->amodel_buf);
	kfree(dbd2);
}

static int
dbmd2_i2c_remove(struct i2c_client *client)
{
	struct dbd2_data *dbd2 = i2c_get_clientdata(client);

	i2c_set_clientdata(client, NULL);
	dbmd2_common_remove(dbd2);

	return 0;
}

static int
dbmd2_platform_probe(struct platform_device *pdev)
{
	struct device_node *np;
	struct dbd2_data *dbd2;
	int ret;

	dbd2 = kzalloc(sizeof(*dbd2), GFP_KERNEL);
	if (dbd2 == NULL) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	dbd2->dev = &pdev->dev;
	np = dbd2->dev->of_node;

	ret = of_property_read_string(np, "uart_device", &dbd2->uart_dev);
	if (ret && ret != -EINVAL) {
		dev_err(dbd2->dev, "invalid 'uart_device'\n");
		goto err_kfree;
	}

	ret = dbmd2_uart_open(dbd2);
	if (ret < 0) {
		dev_err(dbd2->dev, "UART initialization failed\n");
		//goto err_kfree;
	}

	return 0;

err_kfree:
	kfree(dbd2);

	return ret;
}

static int
dbmd2_platform_remove(struct platform_device *pdev)
{
	struct dbd2_data *dbd2 = dev_get_drvdata(&pdev->dev);

	dbmd2_uart_close(dbd2);
	dbmd2_common_remove(dbd2);

	return 0;
}

static const struct of_device_id dbmd2_of_match[] = {
	{ .compatible = "dspg,dbmd2", },
	{},
};
MODULE_DEVICE_TABLE(of, dbmd2_of_match);

static const struct i2c_device_id dbmd2_i2c_id[] = {
	{ "dbmd2", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dbmd2_i2c_id);

static struct i2c_driver dbmd2_i2c_driver = {
	.driver = {
		.name = "dbmd2",
		.owner = THIS_MODULE,
		.of_match_table = dbmd2_of_match,
	},
	.probe =    dbmd2_i2c_probe,
	.remove =   dbmd2_i2c_remove,
	.id_table = dbmd2_i2c_id,
};

static struct of_device_id dbmd2_of_uart_match[] = {
	{ .compatible = "dspg,dbmd2-uart", 0 },
	{ }
};
MODULE_DEVICE_TABLE(of, dbmd2_of_uart_match);

static struct platform_driver dbmd2_platform_driver = {
	.driver = {
		.name = "dbmd2",
		.owner = THIS_MODULE,
		.of_match_table = dbmd2_of_uart_match,
	},
	.probe =    dbmd2_platform_probe,
	.remove =   dbmd2_platform_remove,
};

static int __init dbmd2_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&dbmd2_i2c_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&dbmd2_platform_driver);
	if (ret)
		i2c_del_driver(&dbmd2_i2c_driver);

	return ret;
}
module_init(dbmd2_modinit);

static void __exit dbmd2_exit(void)
{
	platform_driver_unregister(&dbmd2_platform_driver);
	i2c_del_driver(&dbmd2_i2c_driver);
}
module_exit(dbmd2_exit);

MODULE_FIRMWARE(DBD2_FIRMWARE_NAME);
MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION("DSPG DBD2 Voice trigger codec driver");
MODULE_LICENSE("GPL");
