// SPDX-License-Identifier: GPL-2.0-only
/*
 * MSP Watchdog chip driver.
 *
 * Copyright (C) 2015 MSC Technologies, Design Center Aachen
 * Copyright (c) 2010 - 2023 A. Eberle GmbH & Co. KG.
 *
 * Author: Dirk Servos <dservos@msc-technologies.eu>
 * Author: Johannes Eigner <johannes.eigner@a-eberle.de>
 * Author: Stephan Wurm <stephan.wurm@a-eberle.de>
 *
 * The MSP subdevice 0 is a watchdog
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>

/*
 * register set of msp subdevice 0
 */
#define MSP0_SUDEVICE		0x00
#define MSP0_VERSION		0x01
#define MSP0_CTRL		0x02
#define MSP0_STATUS		0x03
#define MSP0_WDT_TO_LOW		0x04
#define MSP0_WDT_TO_MID		0x05
#define MSP0_WDT_TO_HIGH	0x06
#define MSP0_SPARE07		0x07
#define MSP0_WDT_WARN_LOW	0x08
#define MSP0_WDT_WARN_MID	0x09
#define MSP0_WDT_WARN_HIGH	0x0A
#define MSP0_WDT_TRIGGER	0x0B
#define MSP0_REG_CNT		0x0C

#define MSP_WDT_WARN_ENA	0x40
#define MSP_WDT_WARN_FLAG	0x02
#define MSP_WDT_OCCURED		0x01

#define MSP_WDT_TIMEOUT_DEFAULT		120U
#define MSP_WDT_PRETIMEOUT_DEFAULT	90U
#define MSP_WDT_TIMEOUT_MIN		1U
#define MSP_WDT_TIMEOUT_MAX		86400U

// Convert seconds to 8ms ticks
#define MSP_WDT_SECS_TO_TICKS(s)	(s * 125)

#define MSP_WDT_VERSION		"3.1.1"

struct msp_wdt_data {
	struct watchdog_device wdd;
	struct i2c_client *client;
};

static int __msp_wdt_write_reg(struct i2c_client *client,
			       unsigned int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret < 0) {
		dev_err(&client->dev, "failed to write reg: %d, error: %d\n",
			reg, ret);
		return ret;
	}
	return 0;
}

static int msp_wdt_ping(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return __msp_wdt_write_reg(client, MSP0_WDT_TRIGGER, 0x55);
}

static int __msp_wdt_set_timeout(struct watchdog_device *wdd,
				 unsigned int new_timeout)
{
	int tick_timeout;
	int ret;

	struct i2c_client *client = to_i2c_client(wdd->parent);

	dev_info(&client->dev, "set watchdog timeout to %d sec\n", new_timeout);

	tick_timeout = MSP_WDT_SECS_TO_TICKS(new_timeout);
	if (__msp_wdt_write_reg(client, MSP0_WDT_TO_LOW, (u8)(tick_timeout & 0xff)))
		return ret;
	if (__msp_wdt_write_reg(client, MSP0_WDT_TO_MID, (u8)((tick_timeout >> 8) & 0xff)))
		return ret;
	if (__msp_wdt_write_reg(client, MSP0_WDT_TO_HIGH, (u8)((tick_timeout >> 16) & 0xff)))
		return ret;
	return 0;
}

static int msp_wdt_set_timeout(struct watchdog_device *wdd,
			       unsigned int new_timeout)
{
	int ret;
	unsigned int actual;

	actual = min(new_timeout, MSP_WDT_TIMEOUT_MAX);
	ret = __msp_wdt_set_timeout(wdd, actual);
	if (ret)
		goto out;
	wdd->timeout = actual;
out:
	return ret;
}

static int msp_wdt_set_pretimeout(struct watchdog_device *wdd,
				  unsigned int new_pretimeout)
{
	int tick_timeout;
	int ret;
	u8 status;

	struct i2c_client *client = to_i2c_client(wdd->parent);

	dev_info(&client->dev, "set watchdog pretimeout to %d sec\n", new_pretimeout);

	tick_timeout = MSP_WDT_SECS_TO_TICKS(new_pretimeout);
	if (__msp_wdt_write_reg(client, MSP0_WDT_WARN_LOW, (u8)(tick_timeout & 0xff)))
		return ret;
	if (__msp_wdt_write_reg(client, MSP0_WDT_WARN_MID, (u8)((tick_timeout >> 8) & 0xff)))
		return ret;
	if (__msp_wdt_write_reg(client, MSP0_WDT_WARN_HIGH, (u8)((tick_timeout >> 16) & 0xff)))
		return ret;

	// Update WARN_ENA flag
	ret = i2c_smbus_read_byte_data(client, MSP0_STATUS);
	if (ret < 0) {
		dev_err(&client->dev, "%s failed, reg: %d, error: %d\n",
			__func__, MSP0_STATUS, ret);
		return ret;
	}
	status = (u8)ret;
	if (new_pretimeout) {
		// Enable pre-timeout interrupt
		status |= MSP_WDT_WARN_ENA;
	} else {
		// Disable pre-timeout interrupt
		status &= ~MSP_WDT_WARN_ENA;
	}
	ret = __msp_wdt_write_reg(client, MSP0_STATUS, status);
	if (ret)
		return ret;
	wdd->pretimeout = new_pretimeout;
	return 0;
}

/*
 * Set timeout and pretimeout to current values
 */
static int msp_wdt_set_timeouts(struct watchdog_device *wdd)
{
	int ret;

	if (wdd->timeout) {
		ret = __msp_wdt_set_timeout(wdd, wdd->timeout);
		if (ret)
			goto out;
	}
	ret = msp_wdt_set_pretimeout(wdd, wdd->pretimeout);
out:
	return ret;
}

static int msp_wdt_start(struct watchdog_device *wdd)
{
	return msp_wdt_set_timeouts(wdd);
}

static const struct watchdog_info msp_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_PRETIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "MSP Watchdog",
};

static const struct watchdog_ops msp_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= msp_wdt_start,
	.ping		= msp_wdt_ping,
	.set_timeout	= msp_wdt_set_timeout,
	.set_pretimeout	= msp_wdt_set_pretimeout,
};


/*
 * Threaded IRQ handler and this can (and will) sleep.
 */
static irqreturn_t msp_wdt_irq_handler(int irq, void *dev_id)
{
	struct msp_wdt_data *wdev = dev_id;
	struct watchdog_device *wdd = &wdev->wdd;
	struct i2c_client *client = wdev->client;
	int ret;
	u8 status;

	ret = i2c_smbus_read_byte_data(client, MSP0_STATUS);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s failed, reg: %d, error: %d\n", __func__,
			MSP0_STATUS, ret);
		return IRQ_NONE;
	}
	status = (u8)ret;

	if (status & MSP_WDT_WARN_FLAG) {
		if (!(status & MSP_WDT_WARN_ENA)) {
			dev_err(&client->dev,
				"watchdog warning without warn enable!\n");
			return IRQ_NONE;
		}
		// Clear WARN_FLAG, preventing IRQ flooding
		status &= ~MSP_WDT_WARN_FLAG;
		__msp_wdt_write_reg(client, MSP0_STATUS, status);
		watchdog_notify_pretimeout(wdd);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static int __msp_wdt_pretimeout_invalid(struct watchdog_device *wdd,
					unsigned int new_pretimeout)
{
	return new_pretimeout > wdd->timeout;
}

static int msp_wdt_init_pretimeout(struct watchdog_device *wdd,
				   struct device *dev)
{
	unsigned int t = 0;
	int ret = 0;

	/* try to get the pretimeout_sec property */
	if (dev && dev->of_node &&
	    of_property_read_u32(dev->of_node, "pretimeout-sec", &t) == 0) {
		if (!__msp_wdt_pretimeout_invalid(wdd, t)) {
			wdd->pretimeout = t;
			return 0;
		}
		dev_err(dev, "DT supplied pretimeout (%u) out of range\n", t);
		ret = -EINVAL;
	}

	if (ret < 0 && wdd->pretimeout)
		dev_warn(dev, "falling back to default pretimeout (%u)\n",
			wdd->pretimeout);

	return ret;
}

/**
 * msp_wdt_i2c_init() - Initialize I2C device
 *
 * @client I2C slave device
 * @wdev MSP Watchdog device
 *
 * A zero is returned on success or -ENODEV if provided parameter is invalid.
 */
static int msp_wdt_i2c_init(struct i2c_client *client, struct msp_wdt_data *wdev)
{
	struct watchdog_device *wdd;
	int ret;
	u8 status;

	if (!client)
		return -ENODEV;

	if (!wdev)
		return -ENODEV;

	wdd = &wdev->wdd;

	i2c_set_clientdata(client, wdev);

	ret = i2c_smbus_read_byte_data(client, MSP0_VERSION);
	if (ret < 0)
		return ret;
	dev_info(&client->dev, "MSP-VERSION: V%d\n", ret);
	dev_info(&client->dev, "driver version: %s\n", MSP_WDT_VERSION);

	// clear MSP_WDT_OCCURED flag if set
	ret = i2c_smbus_read_byte_data(client, MSP0_STATUS);
	if (ret < 0) {
		dev_err(&client->dev, "%s failed, reg: %d, error: %d\n",
			__func__, MSP0_STATUS, ret);
		return ret;
	}
	status = (u8)ret;
	if (status & MSP_WDT_OCCURED) {
		dev_info(&client->dev, "watchdog timeout occurred!\n");
		wdd->bootstatus = WDIOF_CARDRESET;
		status &= ~MSP_WDT_OCCURED;
		ret = __msp_wdt_write_reg(client, MSP0_STATUS, status);
		if (ret)
			return ret;
	}

	return 0;
}

static int msp_wdt_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct msp_wdt_data *wdev;
	struct watchdog_device *wdd;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	wdev = devm_kzalloc(&client->dev, sizeof(*wdev), GFP_KERNEL);
	if (!wdev)
		return -ENOMEM;

	wdd				= &wdev->wdd;
	wdd->info			= &msp_wdt_info;
	wdd->ops			= &msp_wdt_ops;
	wdd->min_timeout		= MSP_WDT_TIMEOUT_MIN;
	wdd->max_timeout		= MSP_WDT_TIMEOUT_MAX;
	wdd->max_hw_heartbeat_ms	= MSP_WDT_TIMEOUT_MAX * 1000;
	wdd->timeout			= MSP_WDT_TIMEOUT_DEFAULT;
	wdd->pretimeout			= MSP_WDT_PRETIMEOUT_DEFAULT;
	wdd->parent			= dev;
	wdd->status			= WATCHDOG_NOWAYOUT_INIT_STATUS;

	wdev->client = client;

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
		msp_wdt_irq_handler,
		IRQF_SHARED | IRQF_ONESHOT,
		client->name, wdev);
	if (ret) {
		dev_err(&client->dev, "Unable to claim irq %d; error %d\n",
			client->irq, ret);
		return ret;
	}

	ret = msp_wdt_i2c_init(client, wdev);
	if (ret)
		return ret;

	watchdog_set_drvdata(wdd, wdev);
	watchdog_init_timeout(wdd, 0, dev);
	msp_wdt_init_pretimeout(wdd, dev);

	ret = msp_wdt_set_timeouts(wdd);
	if (ret)
		return ret;

	set_bit(WDOG_HW_RUNNING, &wdd->status);

	return watchdog_register_device(wdd);
}

static void msp_wdt_remove(struct i2c_client *client)
{
	struct msp_wdt_data *wdev = i2c_get_clientdata(client);

	watchdog_unregister_device(&wdev->wdd);
}

static const struct i2c_device_id msp_wdt_id[] = {
	{ "msp_wdt" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, msp_wdt_id);

static const struct of_device_id of_msp_wdt_match[] = {
	{ .compatible = "msc,msp-wdt", },
	{},
};
MODULE_DEVICE_TABLE(of, of_msp_wdt_match);

static struct i2c_driver msp_wdt_driver = {
	.driver   = {
		.name    = "msp-wdt",
		.owner   = THIS_MODULE,
		.of_match_table = of_match_ptr(of_msp_wdt_match),
	},
	.probe    = msp_wdt_probe,
	.remove   = msp_wdt_remove,
	.id_table = msp_wdt_id,
};

module_i2c_driver(msp_wdt_driver);

MODULE_AUTHOR("Stephan Wurm <stephan.wurm@a-eberle.de");
MODULE_DESCRIPTION("MSP Watchdog driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(MSP_WDT_VERSION);
