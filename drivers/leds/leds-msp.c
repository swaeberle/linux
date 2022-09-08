// SPDX-License-Identifier: GPL-2.0-only
/*
 * MSP LED chip driver.
 *
 * Copyright (C) 2015 MSC Technologies, Design Center Aachen
 * Copyright (c) 2010 - 2022 A. Eberle GmbH & Co. KG.
 *
 * Author: Dirk Servos <dservos@msc-technologies.eu>
 * Author: Johannes Eigner <johannes.eigner@a-eberle.de>
 *
 * The MSP subdevice 2 is a programmable LED controller that can drive
 * 6 on/off LEDs, 1 RGB LED, and 4 GPIOs
 *
 * Each RGB color appear as a indicator_xxx led an can individual control the brightness from 0..255
 *
 * The on/off LEDsis do hardware-assisted blink synchronously.
 *  If a LED blinks or not can be configured individually
 * but thhey share the same blinker.
 * The 6 Leds have a programmable 'on' and 'off' time as milliseconds.
 * The on/off time can be set from 50ms to 1070ms.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/leds-msp.h>
#include <linux/of.h>



/*
 * register set of msp subdevice 2
 */
#define MSP_SUDEVICE		0x00
#define MSP_VERSION		0x01
#define MSP_SPARE_02H		0x02
#define MSP_STATUS		0x03
#define MSP_LED_TON		0x04
#define MSP_LED_TOFF		0x05
#define MSP_LED_BLINK		0x06
#define MSP_LED_STATE		0x07
#define MSP_PWM_RED		0x08
#define MSP_PWM_GREEN		0x09
#define MSP_PWM_BLUE		0x0A
#define MSP_GPIO_DIR		0x0B
#define MSP_GPIO_IN		0x0C
#define MSP_GPIO_OUT		0x0D

#define	MSP_REG_CNT		0x0E



#define NUM_LEDS 16 /*(6+4+3)*/
#define GPIOBASE 8

struct msp_chip {
	int			reg_set;/* One bit per register where
					 * a '1' means the register
					 * should be written
					 */
	u8			reg_file[MSP_REG_CNT];

	int ontime, offtime;

	struct i2c_client	*client;
	struct work_struct	work;
	spinlock_t		lock;

	struct msp_led {
		struct msp_chip	*chip;
		struct led_classdev	led_cdev;
		int			num;
		int			blink;	/* Set if hardware-blinking */
	} leds[NUM_LEDS];
#ifdef CONFIG_GPIOLIB
	struct gpio_chip		gpio;
	const char			*gpio_name[NUM_LEDS];
	int				gpio_map[NUM_LEDS];
#endif
};




/* Write all needed register of the msp */
static void msp_work(struct work_struct *work)
{
	struct msp_chip *msp = container_of(work, struct msp_chip,
						work);
	struct i2c_client *cl = msp->client;
	int set;
	u8 file[MSP_REG_CNT];
	int r;

	spin_lock_irq(&msp->lock);
	set = msp->reg_set;
	memcpy(file, msp->reg_file, MSP_REG_CNT);
	msp->reg_set = 0;
	spin_unlock_irq(&msp->lock);

	for (r = 0; r < MSP_REG_CNT; r++)
		if (set & (1<<r))
			i2c_smbus_write_byte_data(cl, r, file[r]);
}

static void msp_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct msp_led *led = container_of(led_cdev, struct msp_led, led_cdev);
	struct msp_chip *msp = led->chip;


	led->led_cdev.brightness = brightness;

	switch (led->num) {
	case 12: // RED
		msp->reg_file[MSP_PWM_RED] = brightness;
		msp->reg_set = (1<<MSP_PWM_RED);
		break;
	case 13:// GREEN
		msp->reg_file[MSP_PWM_GREEN] = brightness;
		msp->reg_set = (1<<MSP_PWM_GREEN);
		break;
	case 14: // BLUE
		msp->reg_file[MSP_PWM_BLUE] = brightness;
		msp->reg_set = (1<<MSP_PWM_BLUE);
		break;
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		if (brightness)
			msp->reg_file[MSP_LED_STATE] |= (1<<(led->num));
		else
			msp->reg_file[MSP_LED_STATE] &=  ~(1<<(led->num));
		msp->reg_file[MSP_LED_BLINK] &=  ~(1<<(led->num));
		msp->reg_set = (1<<MSP_LED_STATE) | (1<<MSP_LED_BLINK);
		break;
	}

	if (msp->reg_set)
		schedule_work(&msp->work);
}

static int msp_blink_set(struct led_classdev *led_cdev, unsigned long *delay_on,
			 unsigned long *delay_off)
{
	struct msp_led *led = container_of(led_cdev, struct msp_led, led_cdev);
	struct msp_chip *msp = led->chip;

	if (!msp->ontime)
		msp->ontime = 500; // 500ms default
	if (!msp->offtime)
		msp->offtime = 500; // 500ms default

	if (*delay_on != 0) {
		if (((*delay_on) < 50) || ((*delay_on) > 1070))
			goto error;
		else
			msp->ontime = *delay_on;
	}

	if (*delay_off != 0) {
		if (((*delay_off) < 50) || ((*delay_off) > 1070))
			goto error;
		else
			msp->offtime = *delay_off;
	}

	switch (led->num) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		msp->reg_file[MSP_LED_TON] = (msp->ontime-50)>>2;
		msp->reg_file[MSP_LED_TOFF] = (msp->offtime-50)>>2;
		msp->reg_file[MSP_LED_BLINK] |= (1<<(led->num));
		msp->reg_file[MSP_LED_STATE] |= (1<<(led->num));
		msp->reg_set = (1<<MSP_LED_TON) | (1<<MSP_LED_TOFF) | (1<<MSP_LED_BLINK) |
			(1<<MSP_LED_STATE);
		break;
	}

	if (msp->reg_set)
		schedule_work(&msp->work);

	*delay_on = msp->ontime;
	*delay_off = msp->offtime;
	return 0;
error:
	return -EINVAL;
}

#ifdef CONFIG_GPIOLIB
static void msp_gpio_set_value(struct gpio_chip *gc, unsigned int offset, int val)
{
	struct msp_chip *msp = container_of(gc, struct msp_chip, gpio);
	unsigned long flags;
	int gpio;

	spin_lock_irqsave(&msp->lock, flags);

	gpio = (msp->gpio_map[offset])-GPIOBASE;

	if ((gpio >= 0) && (gpio < 4)) {
		msp->reg_file[MSP_GPIO_DIR] |= (1<<gpio);
		if (val)
			msp->reg_file[MSP_GPIO_OUT] |= (1<<gpio);
		else
			msp->reg_file[MSP_GPIO_OUT] &=  ~(1<<gpio);

		msp->reg_set = (1<<MSP_GPIO_DIR) | (1<<MSP_GPIO_OUT);
	}

	spin_unlock_irqrestore(&msp->lock, flags);

	if (msp->reg_set)
		schedule_work(&msp->work);
}

static int msp_gpio_get_value(struct gpio_chip *gc, unsigned int offset)
{
	struct msp_chip *msp = container_of(gc, struct msp_chip, gpio);
	struct i2c_client *cl = msp->client;
	int gpio;
	int reg;
	int ret = 0;

	gpio = (msp->gpio_map[offset])-GPIOBASE;

	if ((gpio >= 0) && (gpio < 4)) {
		reg = i2c_smbus_read_byte_data(cl, MSP_GPIO_IN);
		ret = !!(reg & (1 << gpio));
	}
	return ret;
}

static int msp_gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int val)
{
	msp_gpio_set_value(gc, offset, val);
	return 0;
}

static int msp_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct msp_chip *msp = container_of(gc, struct msp_chip, gpio);
	unsigned long flags;
	int gpio;

	spin_lock_irqsave(&msp->lock, flags);

	gpio = (msp->gpio_map[offset])-GPIOBASE;

	if ((gpio >= 0) && (gpio < 4)) {
		msp->reg_file[MSP_GPIO_DIR] &=  ~(1<<gpio);
		msp->reg_set = (1<<MSP_GPIO_DIR);
	}
	spin_unlock_irqrestore(&msp->lock, flags);

	if (msp->reg_set)
		schedule_work(&msp->work);
	return 0;
}

static int msp_probe_gpios(struct i2c_client *client, struct msp_chip *msp,
			   struct msp_platform_data *pdata)
{
	int err;
	int i = 0;
	int gpios = 0;

	for (i = 0; i < NUM_LEDS; i++)
		if (pdata->leds.leds[i].name && pdata->leds.leds[i].flags) {
			/* Configure as a gpio */
			msp->gpio_name[gpios] = pdata->leds.leds[i].name;
			msp->gpio_map[gpios] = i;
			gpios++;
		}

	if (!gpios)
		return 0;

	msp->gpio.label = "gpio-msp";
	msp->gpio.names = msp->gpio_name;
	msp->gpio.ngpio = gpios;
	msp->gpio.base = pdata->gpio_base;
	msp->gpio.owner = THIS_MODULE;

	msp->gpio.direction_input = msp_gpio_direction_input;
	msp->gpio.direction_output = msp_gpio_direction_output;
	msp->gpio.set = msp_gpio_set_value;
	msp->gpio.get = msp_gpio_get_value;

	//msp->gpio.dev = &client->dev;
#ifdef CONFIG_OF_GPIO
		msp->gpio.of_node = of_node_get(client->dev.of_node);
#endif
	err = gpiochip_add(&msp->gpio);
	if (err) {
		msp->gpio.ngpio = 0;
		return err;
	}
	if (pdata->setup)
		pdata->setup(msp->gpio.base, msp->gpio.ngpio);
	return 0;
}

static void msp_remove_gpio(struct msp_chip *msp)
{
	if (msp->gpio.ngpio)
		gpiochip_remove(&msp->gpio);
}
#else /* CONFIG_GPIOLIB */
static int msp_probe_gpios(struct i2c_client *client,
			   struct msp_chip *msp,
			   struct msp_platform_data *pdata)
{
	return 0;
}
static void msp_remove_gpio(struct msp_chip *msp)
{
}
#endif /* CONFIG_GPIOLIB */

static struct msp_platform_data *
msp_led_dt_init(struct i2c_client *client)
{
		struct device_node *np = client->dev.of_node, *child;
		struct msp_platform_data *pdata;
		struct led_info *msp_leds;
		int count;

		count = of_get_child_count(np);
		if (!count || count > NUM_LEDS)
			return ERR_PTR(-ENODEV);

		msp_leds = devm_kzalloc(&client->dev, sizeof(struct led_info) * NUM_LEDS,
				GFP_KERNEL);

		if (!msp_leds)
			return ERR_PTR(-ENOMEM);

		for_each_child_of_node(np, child) {
			struct led_info led;
			u32 reg;
			int ret;

			led.name = of_get_property(child, "label", NULL) ? : child->name;
			led.default_trigger = of_get_property(child, "linux,default-trigger", NULL);
			led.flags = 0;
			if (of_property_match_string(child, "compatible", "gpio") >= 0)
				led.flags |= MSP_MAKE_GPIO;
			ret = of_property_read_u32(child, "reg", &reg);
			if (ret != 0 || reg < 0 || reg >= NUM_LEDS)
				continue;

			msp_leds[reg] = led;
		}
		pdata = devm_kzalloc(&client->dev, sizeof(struct msp_platform_data), GFP_KERNEL);

		if (!pdata)
			return ERR_PTR(-ENOMEM);

		pdata->leds.leds = msp_leds;
		pdata->leds.num_leds = NUM_LEDS;
#ifdef CONFIG_GPIOLIB
		pdata->gpio_base = -1;
#endif
		return pdata;
}



static int msp_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msp_chip *msp;
	struct i2c_adapter *adapter;
	struct msp_platform_data *pdata;
	int err;
	int i = 0;

	adapter = to_i2c_adapter(client->dev.parent);

	pdata = dev_get_platdata(&client->dev);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	if (!pdata || pdata->leds.num_leds != NUM_LEDS) {
		pdata = msp_led_dt_init(client);
		if (IS_ERR(pdata)) {
			dev_err(&client->dev, "Need %d entries in platform-data list\n",
				NUM_LEDS);
			return PTR_ERR(pdata);
		}
	}
	msp = devm_kzalloc(&client->dev, sizeof(*msp), GFP_KERNEL);
	if (!msp)
		return -ENOMEM;

	msp->client = client;
	INIT_WORK(&msp->work, msp_work);
	spin_lock_init(&msp->lock);
	i2c_set_clientdata(client, msp);

	for (i = 0; i < NUM_LEDS; i++) {
		struct msp_led *l = msp->leds + i;

		l->chip = msp;
		l->num = i;
		if (pdata->leds.leds[i].name && !pdata->leds.leds[i].flags) {
			l->led_cdev.name = pdata->leds.leds[i].name;
			l->led_cdev.default_trigger = pdata->leds.leds[i].default_trigger;
			l->led_cdev.brightness_set = msp_brightness_set;
			l->led_cdev.blink_set = msp_blink_set;

			err = led_classdev_register(&client->dev, &l->led_cdev);
			if (err < 0)
				goto exit;
		}
	}
	err = msp_probe_gpios(client, msp, pdata);
	if (err)
		goto exit;
	/* set all registers to known state - zero */
	msp->reg_set = 0x3FF0;
	schedule_work(&msp->work);

	return 0;
exit:
	while (i--) {
		if (msp->leds[i].led_cdev.name)
			led_classdev_unregister(&msp->leds[i].led_cdev);
	}
	return err;
}

static void msp_remove(struct i2c_client *client)
{
	int i;
	struct msp_chip *msp = i2c_get_clientdata(client);
	struct msp_led *msp_leds = msp->leds;

	for (i = 0; i < NUM_LEDS; i++) {
		if (msp_leds[i].led_cdev.name)
			led_classdev_unregister(&msp_leds[i].led_cdev);
	}
	msp_remove_gpio(msp);
	cancel_work_sync(&msp->work);
}

static const struct of_device_id of_msp_leds_match[] = {
	{ .compatible = "msc,msp", },
	{},
};
MODULE_DEVICE_TABLE(of, of_msp_leds_match);

static const struct i2c_device_id msp_id[] = {
	{ "msp" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, msp_id);
static struct i2c_driver msp_driver = {
	.driver   = {
		.name    = "leds-msp",
		.owner   = THIS_MODULE,
		.of_match_table = of_msp_leds_match,
	},
	.probe    = msp_probe,
	.remove   = msp_remove,
	.id_table = msp_id,
};

module_i2c_driver(msp_driver);

MODULE_AUTHOR("Dirk Servos <dservos@msc-technologies.eu>");
MODULE_DESCRIPTION("MSP LED/RGB/GPIO driver");
MODULE_LICENSE("GPL");
