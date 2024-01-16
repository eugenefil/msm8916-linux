/*
 * FocalTech ft5436 TouchScreen driver.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>

#define FT5436_MAX_FINGERS	5

#define FT_TOUCH_DOWN		0
#define FT_TOUCH_UP		1
#define FT_TOUCH_ON		2
#define FT_TOUCH_NONE		3

#define FT_REG_ID		0xa3
#define FT_REG_PMODE		0xa5

#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_HIBERNATE	0x03

#define FT_RESET_DELAY_MS	20
#define FT_STARTUP_DELAY_MS	200

struct ft5436_point {
	u8 event_x_hi;
	u8 x_lo;
	u8 id_y_hi;
	u8 y_lo;
	u8 pad[2];
} __packed;

struct ft5436_touch_packet {
	u8 pad[2];
	u8 status;
	struct ft5436_point points[FT5436_MAX_FINGERS];
} __packed;

struct ft5436_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpiod;
};

static int ft5436_i2c_xfer(struct i2c_client *client, u8 *wbuf, u16 wlen,
		u8 *rbuf, u16 rlen)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = wlen,
			.buf = wbuf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = rlen,
			.buf = rbuf,
		},
	};
	int ret, num_msgs = rbuf ? 2 : 1;
	ret = i2c_transfer(client->adapter, msgs, num_msgs);
	if (ret != num_msgs) {
		dev_err(&client->dev, "Failed i2c transfer: %d\n", ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int ft5436_i2c_readreg(struct i2c_client *client, u8 reg,
		u8 *rbuf, u16 rlen)
{
	return ft5436_i2c_xfer(client, &reg, 1, rbuf, rlen);
}

static int ft5436_i2c_writereg(struct i2c_client *client, u8 reg, u8 val)
{
	u8 buf[] = { reg, val };
	return ft5436_i2c_xfer(client, buf, sizeof(buf), NULL, 0);
}

static irqreturn_t ft5436_interrupt(int irq, void *dev_id)
{
	struct ft5436_data *data = dev_id;
	struct input_dev *input = data->input;
	struct i2c_client *client = data->client;
	int ret, i;
	u16 id, x, y, event;
	struct ft5436_touch_packet pkt;
	struct ft5436_point *pt;

	ret = ft5436_i2c_readreg(client, 0x00, (u8 *)&pkt, sizeof(pkt));
	if (ret) {
		dev_err(&client->dev, "Failed to read touch data\n");
		return IRQ_HANDLED;
	}

	for (i = 0; i < ARRAY_SIZE(pkt.points); i++) {
		pt = &pkt.points[i];
		event = pt->event_x_hi >> 6;
		if (event == FT_TOUCH_NONE) /* no more points */
			break;

		id = pt->id_y_hi >> 4;
		x = (pt->event_x_hi & 0x0f) << 8 | pt->x_lo;
		y = (pt->id_y_hi & 0x0f) << 8 | pt->y_lo;
		dev_dbg(&client->dev, "event=%u id=%u x=%u y=%u\n", event, id, x, y);
		input_mt_slot(input, id);
		if (input_mt_report_slot_state(input, MT_TOOL_FINGER,
					event != FT_TOUCH_UP)) {
			touchscreen_report_pos(input, &data->prop, x, y, true);
		}
	}
	input_sync(input);

	return IRQ_HANDLED;
}

static int ft5436_power_on(struct ft5436_data *data)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(data->supplies),
			data->supplies);
	if (ret) {
		dev_err(&data->client->dev,
			"Failed to enable regulators: %d\n", ret);
		return ret;
	}

	gpiod_set_value_cansleep(data->reset_gpiod, 1);
	msleep(FT_RESET_DELAY_MS);
	gpiod_set_value_cansleep(data->reset_gpiod, 0);
	msleep(FT_STARTUP_DELAY_MS);
	return 0;
}

static int ft5436_power_off(struct ft5436_data *data)
{
	int ret;

	ret = regulator_bulk_disable(ARRAY_SIZE(data->supplies),
			data->supplies);
	if (ret) {
		dev_err(&data->client->dev,
			"Failed to disable regulators: %d\n", ret);
		return ret;
	}
	return 0;
}

static int ft5436_input_open(struct input_dev *dev)
{
	struct ft5436_data *data = input_get_drvdata(dev);
	int ret;

	ret = ft5436_power_on(data);
	if (ret) {
		dev_err(&data->client->dev, "Failed to power on while opening device\n");
		return ret;
	}
	enable_irq(data->client->irq);
	return 0;
}

static void ft5436_input_close(struct input_dev *dev)
{
	struct ft5436_data *data = input_get_drvdata(dev);

	disable_irq(data->client->irq);
	ft5436_power_off(data);
}

static int ft5436_init_input_dev(struct ft5436_data *data)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input = input;
	input_set_drvdata(input, data);

	input->name = "FocalTech TouchScreen";
	input->phys = "input/ts";
	input->id.bustype = BUS_I2C;
	input->open = ft5436_input_open;
	input->close = ft5436_input_close;

	input_set_capability(input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_Y);

	touchscreen_parse_properties(input, true, &data->prop);
	if (!data->prop.max_x || !data->prop.max_y) {
		dev_err(dev, "touchscreen-size-x and/or touchscreen-size-y not set in device tree\n");
		return -EINVAL;
	}

	ret = input_mt_init_slots(input, FT5436_MAX_FINGERS,
			INPUT_MT_DIRECT);
	if (ret) {
		dev_err(dev, "Failed to initialize MT slots: %d\n", ret);
		return ret;
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "Failed to register input device\n");
		return ret;
	}

	return 0;
}

static int ft5436_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ft5436_data *data;
	u8 id_val;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "Plain I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);

	data->supplies[0].supply = "vdd";
	data->supplies[1].supply = "iovcc";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(data->supplies),
			data->supplies);
	if (ret) {
		dev_err(dev, "Failed to get regulators: %d\n", ret);
		return ret;
	}

	data->reset_gpiod = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(data->reset_gpiod)) {
		ret = PTR_ERR(data->reset_gpiod);
		dev_err(dev, "Failed to request reset GPIO: %d\n", ret);
		return ret;
	}

	ret = ft5436_power_on(data);
	if (ret) {
		dev_err(dev, "Failed to power on\n");
		return ret;
	}

	/* check controller id */
	ret = ft5436_i2c_readreg(client, FT_REG_ID, &id_val, 1);
	if (ret) {
		dev_err(dev, "Failed to read device id: %d\n", ret);
		ft5436_power_off(data);
		return ret;
	}
	dev_info(dev, "Device ID = 0x%x\n", id_val);

	ret = ft5436_power_off(data);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
			ft5436_interrupt,
			IRQF_ONESHOT | IRQF_NO_AUTOEN,
			dev->driver->name, data);
	if (ret) {
		dev_err(dev, "Failed to request irq: %d\n", ret);
		return ret;
	}

	ret = ft5436_init_input_dev(data);
	if (ret) {
		dev_err(dev, "Failed to init input device\n");
		return ret;
	}

	return 0;
}

static int __maybe_unused ft5436_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5436_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->input->mutex);

	if (input_device_enabled(data->input)) {
		disable_irq(client->irq);
		if (ft5436_i2c_writereg(client, FT_REG_PMODE, FT_PMODE_HIBERNATE))
			dev_warn(&client->dev, "Failed to enable hibernate mode\n");
		ft5436_power_off(data);
	}

	mutex_unlock(&data->input->mutex);
	return 0;
}

static int __maybe_unused ft5436_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5436_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&data->input->mutex);

	if (input_device_enabled(data->input)) {
		ret = ft5436_power_on(data);
		if (ret) {
			dev_err(dev, "Failed to power on while resuming\n");
			goto unlock;
		}
		enable_irq(client->irq);
	}

unlock:
	mutex_unlock(&data->input->mutex);
	return ret;
}

static SIMPLE_DEV_PM_OPS(ft5436_pm_ops, ft5436_suspend, ft5436_resume);

static struct of_device_id ft5436_of_match[] = {
	{ .compatible = "focaltech,5436" },
	{ },
};
MODULE_DEVICE_TABLE(of, ft5436_of_match);

static struct i2c_driver ft5436_driver = {
	.probe_new = ft5436_probe,
	.driver = {
		   .name = "ft5436_ts",
		   .of_match_table = ft5436_of_match,
		   .pm = &ft5436_pm_ops,
	},
};
module_i2c_driver(ft5436_driver);

MODULE_DESCRIPTION("FocalTech ft5436 TouchScreen driver");
MODULE_LICENSE("GPL v2");
