// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 fuel gauge, read only.
 *
 * The gauge runs on the battery and keeps the battery model the vendor
 * firmware loaded into it across reboots. This driver only reads it: the
 * state of charge, voltage, current and temperature words of its SRAM, and
 * the status of the SM5714 charger block next to it. The one write is the
 * SRAM read address. No battery table, limit, reset or charger setting is
 * ever written; the charger keeps its power-on behaviour.
 *
 * Register map and conversions as in Samsung's sm5714_fuelgauge.c and
 * sm5714_charger.c (Galaxy Tab S7 FE kernel source).
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>

#define SM5714_FG_SYSTEM_STATUS		0x10
#define SM5714_FG_INIT_DONE		BIT(12)	/* clear: the gauge lost its battery model */
#define SM5714_FG_NO_BATT_PATH		BIT(15)	/* Samsung reports 0 current then (jig, nENQ4 low) */
#define SM5714_FG_AUX_STAT		0x22
#define SM5714_FG_AUX_X2_5		BIT(10)	/* current reads are x2.5 */
#define SM5714_FG_SRAM_RADDR		0x8c
#define SM5714_FG_SRAM_RDATA		0x8d

#define SM5714_FG_SRAM_SOC		0x00
#define SM5714_FG_SRAM_VBAT		0x03
#define SM5714_FG_SRAM_CURRENT		0x05
#define SM5714_FG_SRAM_TEMPERATURE	0x07
#define SM5714_FG_SRAM_CURRENT_AVG	0x09

#define SM5714_CHG_ADDR			0x49
#define SM5714_CHG_STATUS1		0x0d
#define SM5714_CHG_VBUS_POK		BIT(0)
#define SM5714_CHG_VBUS_OVP		BIT(2)
#define SM5714_CHG_STATUS2		0x0e
#define SM5714_CHG_BAT_ABSENT		BIT(2)
#define SM5714_CHG_CHARGING		BIT(3)
#define SM5714_CHG_TOPOFF		BIT(5)

struct sm5714_fg {
	struct i2c_client *client;
	struct i2c_client *chg;
	struct mutex lock;	/* the SRAM address and data reads are one transaction */
};

static int sm5714_fg_sram(struct sm5714_fg *fg, u8 addr)
{
	int ret;

	mutex_lock(&fg->lock);
	ret = i2c_smbus_write_word_data(fg->client, SM5714_FG_SRAM_RADDR, addr);
	if (!ret)
		ret = i2c_smbus_read_word_data(fg->client, SM5714_FG_SRAM_RDATA);
	mutex_unlock(&fg->lock);

	return ret;
}

/* Sign and magnitude: bit 15 is the sign. */
static int sm5714_fg_signed(int raw, int mag)
{
	return raw & 0x8000 ? -mag : mag;
}

static int sm5714_fg_current_ua(struct sm5714_fg *fg, u8 addr, int *val)
{
	int raw = sm5714_fg_sram(fg, addr);
	int aux = i2c_smbus_read_word_data(fg->client, SM5714_FG_AUX_STAT);
	int sys = i2c_smbus_read_word_data(fg->client, SM5714_FG_SYSTEM_STATUS);
	int ma;

	if (raw < 0)
		return raw;
	if (aux < 0)
		return aux;
	if (sys < 0)
		return sys;
	/* mA = (raw & 0x7fff) * 1000 / 2044; bit 15 set = discharging */
	ma = sm5714_fg_signed(raw, (raw & 0x7fff) * 1000 / 2044);
	if (aux & SM5714_FG_AUX_X2_5)
		ma = ma * 25 / 10;
	if (sys & SM5714_FG_NO_BATT_PATH)
		ma = 0;
	*val = ma * 1000;
	return 0;
}

static int sm5714_fg_status(struct sm5714_fg *fg, int *val)
{
	int st1 = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS1);
	int st2 = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS2);

	if (st1 < 0)
		return st1;
	if (st2 < 0)
		return st2;

	if (st2 & SM5714_CHG_TOPOFF)
		*val = POWER_SUPPLY_STATUS_FULL;
	else if (st2 & SM5714_CHG_CHARGING)
		*val = POWER_SUPPLY_STATUS_CHARGING;
	else if (st1 & SM5714_CHG_VBUS_POK)
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
	return 0;
}

static int sm5714_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5714_fg *fg = power_supply_get_drvdata(psy);
	int raw, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return sm5714_fg_status(fg, &val->intval);
	case POWER_SUPPLY_PROP_PRESENT:
		raw = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS2);
		if (raw < 0)
			return raw;
		val->intval = !(raw & SM5714_CHG_BAT_ABSENT);
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		raw = i2c_smbus_read_byte_data(fg->chg, SM5714_CHG_STATUS1);
		if (raw < 0)
			return raw;
		val->intval = raw & SM5714_CHG_VBUS_OVP ? POWER_SUPPLY_HEALTH_OVERVOLTAGE
						       : POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = i2c_smbus_read_word_data(fg->client, SM5714_FG_SYSTEM_STATUS);
		if (ret < 0)
			return ret;
		if (!(ret & SM5714_FG_INIT_DONE))
			return -ENODATA;	/* no battery model: the number would be meaningless */
		raw = sm5714_fg_sram(fg, SM5714_FG_SRAM_SOC);
		if (raw < 0)
			return raw;
		/* (raw * 10) >> 8 is in 0.1 % */
		val->intval = clamp(((raw * 10) >> 8) / 10, 0, 100);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		raw = sm5714_fg_sram(fg, SM5714_FG_SRAM_VBAT);
		if (raw < 0)
			return raw;
		/* mV = 2700 +/- (raw & 0x7fff) * 10 / 109 */
		val->intval = (2700 + sm5714_fg_signed(raw, (raw & 0x7fff) * 10 / 109)) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return sm5714_fg_current_ua(fg, SM5714_FG_SRAM_CURRENT, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		return sm5714_fg_current_ua(fg, SM5714_FG_SRAM_CURRENT_AVG, &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		raw = sm5714_fg_sram(fg, SM5714_FG_SRAM_TEMPERATURE);
		if (raw < 0)
			return raw;
		/* 0.1 degC = (raw & 0x7fff) * 10 * 2989 >> 19 */
		val->intval = sm5714_fg_signed(raw, ((raw & 0x7fff) * 10 * 2989) >> 19);
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property sm5714_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc sm5714_fg_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sm5714_fg_props,
	.num_properties = ARRAY_SIZE(sm5714_fg_props),
	.get_property = sm5714_fg_get_property,
};

static int sm5714_fg_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = { .fwnode = dev_fwnode(&client->dev) };
	struct device *dev = &client->dev;
	struct power_supply *psy;
	struct sm5714_fg *fg;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;
	fg->client = client;
	ret = devm_mutex_init(dev, &fg->lock);
	if (ret)
		return ret;

	ret = i2c_smbus_read_word_data(client, SM5714_FG_SYSTEM_STATUS);
	if (ret < 0)
		return dev_err_probe(dev, ret, "gauge does not answer\n");
	if (!(ret & SM5714_FG_INIT_DONE))
		dev_warn(dev, "gauge has no battery model (status %#06x): capacity unavailable\n", ret);

	/* The charger block of the same chip, for the charging status only. */
	fg->chg = devm_i2c_new_dummy_device(dev, client->adapter, SM5714_CHG_ADDR);
	if (IS_ERR(fg->chg))
		return dev_err_probe(dev, PTR_ERR(fg->chg), "charger block\n");

	cfg.drv_data = fg;
	psy = devm_power_supply_register(dev, &sm5714_fg_desc, &cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy), "failed to register battery\n");

	return 0;
}

static const struct of_device_id sm5714_fg_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-fuel-gauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_fg_of_match);

static struct i2c_driver sm5714_fg_driver = {
	.driver = {
		.name = "sm5714-fuel-gauge",
		.of_match_table = sm5714_fg_of_match,
	},
	.probe = sm5714_fg_probe,
};
module_i2c_driver(sm5714_fg_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 fuel gauge (read only)");
MODULE_LICENSE("GPL");
