/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MSP Watchdog chip driver.
 *
 * Copyright (C) 2015 MSC Technologies, Design Center Aachen
 * Copyright (c) 2010 - 2023 A. Eberle GmbH & Co. KG.
 *
 * Author: Dirk Servos <dservos@msc-technologies.eu>
 * Author: Johannes Eigner <johannes.eigner@a-eberle.de>
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

#define MSP_WATCHDOG_TIMEOUT_DEFAULT_S		120
#define MSP_WATCHDOG_WARN_TIMEOUT_DEFAULT_S	90
#define MSP_WATCHDOG_TIMEOUT_MIN		10
#define MSP_WATCHDOG_TIMEOUT_MAX		600

struct msp_wdt_data {
	struct watchdog_device wdd;
	int warn_timeout;
	struct i2c_client *client;
	struct work_struct work;
	u8 reg_file[MSP0_REG_CNT];
};

static int msp_wdt_stop(struct watchdog_device *wdd)
{
	// This watchdog can't be stopped
	return 0;
}

static int msp_wdt_ping(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return i2c_smbus_write_byte_data(client, MSP0_WDT_TRIGGER, 0x55);
}

static int msp_wdt_set_timeout(struct watchdog_device *wdd,
			       unsigned int timeout)
{
	int tick_timeout;
	s32 error;
	u8 reg_timeout[3];

	struct i2c_client *client = to_i2c_client(wdd->parent);

	dev_info(&client->dev, "set watchdog timeout to %d sec\n", timeout);

	wdd->timeout = timeout;

	tick_timeout = wdd->timeout * 125; // 1 tick = 8 ms
	reg_timeout[0] = (u8)(tick_timeout & 0xff);
	reg_timeout[1] = (u8)((tick_timeout >> 8) & 0xff);
	reg_timeout[2] = (u8)((tick_timeout >> 16) & 0xff);
	error = i2c_smbus_write_block_data(client, MSP0_WDT_TO_LOW,
		sizeof(reg_timeout), reg_timeout);
	if (error) {
		dev_err(&client->dev,
			"%s failed to write reg: %d, error: %d\n",
			__func__, MSP0_WDT_TO_LOW, error);
		return error;
	}

	tick_timeout = wdd->timeout * 94; // warn = 3/4 of timeout
	reg_timeout[0] = (u8)(tick_timeout & 0xff);
	reg_timeout[1] = (u8)((tick_timeout >> 8) & 0xff);
	reg_timeout[2] = (u8)((tick_timeout >> 16) & 0xff);
	error = i2c_smbus_write_block_data(client, MSP0_WDT_WARN_LOW,
		sizeof(reg_timeout), reg_timeout);
	if (error) {
		dev_err(&client->dev,
			"%s failed to write reg: %d, error: %d\n",
			__func__, MSP0_WDT_WARN_LOW, error);
		return error;
	}

	return 0;
}

static int msp_wdt_start(struct watchdog_device *wdd)
{
	s32 error;
	u8 reg;

	struct i2c_client *client = to_i2c_client(wdd->parent);
	struct msp_wdt_data *msp_wdt = watchdog_get_drvdata(wdd);

	// clear MSP_WDT_OCCURED flag if set
	error = i2c_smbus_read_byte_data(client, MSP0_STATUS);
	if (error < 0) {
		dev_err(&client->dev, "%s failed, reg: %d, error: %d\n",
			__func__, MSP0_STATUS, error);
		return error;
	}
	reg = (u8)error;
	msp_wdt->reg_file[MSP0_STATUS] |= MSP_WDT_WARN_ENA;
	if (reg & MSP_WDT_OCCURED) {
		dev_info(&client->dev, "watchdog timeout occurred !\n");
		reg = msp_wdt->reg_file[MSP0_STATUS] & (~MSP_WDT_OCCURED);
		i2c_smbus_write_byte_data(client, MSP0_STATUS,
			reg);
	}
	set_bit(WDOG_ACTIVE, &wdd->status);

	return msp_wdt_set_timeout(wdd, wdd->timeout);
}

static const struct watchdog_info msp_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity = "MSP Watchdog",
};

static const struct watchdog_ops msp_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= msp_wdt_start,
	.stop		= msp_wdt_stop,
	.ping		= msp_wdt_ping,
	.set_timeout	= msp_wdt_set_timeout,
};

// TODO Add ATTRIBUTE_GROUPS and DEVICE_ATTR here


/*
 * Threaded IRQ handler and this can (and will) sleep.
 */
static irqreturn_t msp_wdt_irq_handler(int irq, void *dev_id)
{
	struct msp_wdt_data *msp_wdt = dev_id;
	s32 error;
	u8 reg;

	error = i2c_smbus_read_byte_data(msp_wdt->client, MSP0_STATUS);
	if (error < 0) {
		dev_err(&msp_wdt->client->dev,
			"%s failed, reg: %d, error: %d\n", __func__,
			MSP0_STATUS, error);
		return IRQ_NONE;
	}
	reg = (u8)error;

	if (reg & MSP_WDT_WARN_FLAG) {
		if (!(reg & MSP_WDT_WARN_ENA)) {
			dev_warn(&msp_wdt->client->dev,
				 "watchdog warning without warn enable !\n");
			return IRQ_NONE;
		}
		dev_warn(&msp_wdt->client->dev, "watchdog warning !\n");
		reg = msp_wdt->reg_file[MSP0_STATUS] & (~MSP_WDT_WARN_FLAG);
		i2c_smbus_write_byte_data(msp_wdt->client, MSP0_STATUS, reg);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/**
 * msp_wdt_dt_init() - Read timeout values from devicetree
 *
 * @client I2C slave device
 * @msp_wdt MSP Watchdog data
 *
 * A zero is returned on success or -ENODEV if provided parameter is invalid.
 */
static int
msp_wdt_dt_init(struct i2c_client *client, struct msp_wdt_data *msp_wdt)
{
	struct device_node *np = client->dev.of_node;
	int ret;
	int timeout;
	int warn_timeout;

	if (!client)
		return -ENODEV;

	if (!msp_wdt)
		return -ENODEV;

	watchdog_set_drvdata(&msp_wdt->wdd, msp_wdt);

	ret = of_property_read_u32(np, "wdg-timeout-sec", &timeout);
	if (ret < 0) {
		dev_info(&client->dev,
			"Property 'wdg-timeout-sec' not found, using default.\n");
		timeout = MSP_WATCHDOG_TIMEOUT_DEFAULT_S;
	}
	msp_wdt->wdd.timeout = timeout;
	dev_info(&client->dev, "wdg timeout set to %d sec\n", timeout);

	ret = of_property_read_u32(np, "wdg-warn-sec", &warn_timeout);
	if (ret < 0) {
		dev_info(&client->dev,
			 "Property 'wdg-warn-sec' not found, using default.\n");
		warn_timeout = MSP_WATCHDOG_WARN_TIMEOUT_DEFAULT_S;
	}
	msp_wdt->warn_timeout = warn_timeout;
	dev_info(&client->dev, "wdg warn timeout set to %d sec\n",
		 warn_timeout);
	return 0;
}

static int msp_wdt_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct msp_wdt_data *msp_wdt;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	msp_wdt = devm_kzalloc(&client->dev, sizeof(*msp_wdt), GFP_KERNEL);
	if (!msp_wdt)
		return -ENOMEM;

	msp_wdt->wdd.info		= &msp_wdt_info;
	msp_wdt->wdd.ops		= &msp_wdt_ops;
	msp_wdt->wdd.min_timeout	= MSP_WATCHDOG_TIMEOUT_MIN;
	msp_wdt->wdd.max_timeout	= MSP_WATCHDOG_TIMEOUT_MAX;
	msp_wdt->wdd.timeout		= MSP_WATCHDOG_TIMEOUT_DEFAULT_S;
	msp_wdt->wdd.parent		= &client->dev;
	msp_wdt->wdd.status		= WDOG_ACTIVE;

	msp_wdt->client = client;
	msp_wdt->warn_timeout		= MSP_WATCHDOG_WARN_TIMEOUT_DEFAULT_S;
	// TODO maybe use wdd.pretimeout

	msp_wdt_dt_init(client, msp_wdt);

	i2c_set_clientdata(client, msp_wdt);

	err = i2c_smbus_read_byte_data(client, MSP0_VERSION);
	if (err < 0)
		return err;
	dev_info(&client->dev, "MSP-VERSION: V%d\n", err);

	//INIT_WORK(&msp_wdt->work, msp_wdt_work);

	msp_wdt_start(&msp_wdt->wdd);

	err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
		msp_wdt_irq_handler,
		IRQF_SHARED | IRQF_ONESHOT,
		client->name, msp_wdt);
	if (err) {
		dev_err(&client->dev, "Unable to claim irq %d; error %d\n",
			client->irq, err);
		return err;
	}

	err = watchdog_register_device(&msp_wdt->wdd);
	return err;
}

static void msp_wdt_remove(struct i2c_client *client)
{
	struct msp_wdt_data *msp_wdt = i2c_get_clientdata(client);

	watchdog_unregister_device(&msp_wdt->wdd);
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

MODULE_AUTHOR("Dirk Servos <dservos@msc-technologies.eu");
MODULE_DESCRIPTION("MSP Watchdog driver");
MODULE_LICENSE("GPL");
