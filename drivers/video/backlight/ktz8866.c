// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight driver for the Kinetic KTZ8866
 * Jianhua Lu <lujianhua000@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>

#define KTZ8866_ENABLE_REG           0x08
#define KTZ8866_BRIGHT_LB_REG        0x04
#define KTZ8866_BRIGHT_HB_REG        0x05

#define KTZ8866_ON_CMD               0x5F
#define KTZ8866_OFF_CMD              0x1F

#define KTZ8866_DEF_BRIGHT           1800
#define KTZ8866_MAX_BRIGHT           2047

/* Helper */
#define HIGH_BYTE(x)                 (u8)((x >> 3) & 0xFF)
#define LOW_BYTE(x)                  (u8)(x & 0x7)

struct ktz8866 {
	struct i2c_client *client;
	struct backlight_device *backlight;
	bool status;
};

int ktz8866_write(struct ktz8866 *ktz, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(ktz->client, reg, data);
}

static int ktz8866_backlight_update_status(struct backlight_device *backlight)
{
	struct ktz8866 *ktz = bl_get_data(backlight);
	int brightness = backlight_get_brightness(backlight);

	if (!ktz->status && brightness > 0) {
		ktz8866_write(ktz, KTZ8866_ENABLE_REG, KTZ8866_ON_CMD);
		ktz->status = 1;
	} else if (brightness == 0) {
		ktz8866_write(ktz, KTZ8866_ENABLE_REG, KTZ8866_OFF_CMD);
		ktz->status = 0;
		msleep(10);
	}

	/* Set brightness */
	ktz8866_write(ktz, KTZ8866_BRIGHT_HB_REG, HIGH_BYTE(brightness));
	ktz8866_write(ktz, KTZ8866_BRIGHT_LB_REG, LOW_BYTE(brightness));

	return 0;
}

static const struct backlight_ops ktz8866_backlight_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= ktz8866_backlight_update_status,
};

static int ktz8866_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct backlight_device *backlight;
	struct backlight_properties props;
	struct ktz8866 *ktz;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&client->dev,
			 "ktz8866 I2C adapter doesn't support I2C_FUNC_SMBUS_BYTE\n");
		return -EIO;
	}

	ktz = devm_kzalloc(&client->dev, sizeof(*ktz), GFP_KERNEL);
	if (!ktz)
		return -ENOMEM;

	ktz->client = client;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = KTZ8866_MAX_BRIGHT;
	props.brightness = clamp_t(unsigned int, KTZ8866_DEF_BRIGHT, 0,
				   props.max_brightness);

	backlight = devm_backlight_device_register(&client->dev, "ktz8866-backlight",
					      &client->dev, ktz, &ktz8866_backlight_ops,
					      &props);
	if (IS_ERR(backlight)) {
		dev_err(&client->dev, "failed to register backlight\n");
		return PTR_ERR(backlight);
	}

	backlight_update_status(backlight);

	i2c_set_clientdata(client, backlight);

	return 0;
}

static int ktz8866_remove(struct i2c_client *client)
{
	struct backlight_device *backlight = i2c_get_clientdata(client);

	backlight->props.brightness = 0;
	backlight_update_status(backlight);

	return 0;
}

static const struct i2c_device_id ktz8866_ids[] = {
	{ "ktz8866", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ktz8866_ids);

static const struct of_device_id ktz8866_match_table[] = {
	{ .compatible = "kinetic,ktz8866",},
	{}
};

static struct i2c_driver ktz8866_driver = {
	.driver = {
		.name = "ktz8866",
		.of_match_table = ktz8866_match_table,
	},
	.probe = ktz8866_probe,
	.remove = ktz8866_remove,
	.id_table = ktz8866_ids,
};

module_i2c_driver(ktz8866_driver);

MODULE_AUTHOR("Jianhua Lu <lujianhua000@gmail.com>");
MODULE_DESCRIPTION("Kinetic KTZ8866 Backlight Driver");
MODULE_LICENSE("GPL");
