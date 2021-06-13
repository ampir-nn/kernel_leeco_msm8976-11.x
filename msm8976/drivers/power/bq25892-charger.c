/* Copyright (c) 2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/qpnp/qpnp-adc.h>
#include "bq25892-charger.h"
#include <linux/time.h>

/* Mask/Bit helpers */
#define _BQ25892_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define BQ25892_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_BQ25892_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

#define EN_BY_PIN_HIGH_ENABLE			0x40
#define EN_BY_PIN_LOW_ENABLE			0x60

#define DEFAULT_INPUT_CURRENT			100


/* Command registers */
#define CMD_CHG_REG				0x32
#define CMD_OTG_EN_BIT				BIT(0)

/* constants */
#define BQ25892_CHG_FAST_MIN_MA			1
#define BQ25892_CHG_FAST_MAX_MA			5056

#define CHG_REG_00				0x0
#define INPUT_CURRENT_LIMIT_MASK		BQ25892_MASK(5, 0)
#define EN_ILIM_BIT				BIT(6)

#define CHG_REG_01				0x1
#define BHOT_MASK				BQ25892_MASK(7, 6)
#define VINDPM_OS_BIT				BIT(1) | BIT(3)
#define VINDPM_OS_MASK				BQ25892_MASK(4, 0)
#define CHG_REG_02				0x2
#define CONV_START_BIT				BIT(7)
#define CONV_START_MASK				BIT(7)
#define AUTO_DPDM_EN_BIT 			0x0
#define AUTO_DPDM_EN_MASK			BIT(0)
#define MAXC_EN_BIT				0x0
#define MAXC_EN_MASK				BIT(2)
#define HVDCP_EN_BIT				0x0
#define HVDCP_EN_MASK				BIT(3)
#define ICO_EN_BIT				0x0
#define ICO_EN_MASK				BIT(4)

#define CHG_REG_03				0x3
#define CHG_CONFIG_EN_BIT			BIT(4)
#define CHG_CONFIG_DISEN_BIT			0x0
#define CHG_CONFIG_MASK				BIT(4)
#define WD_RST_BIT				BIT(6)

#define CHG_REG_04				0x4
#define FAST_CHG_CURRENT_MASK			BQ25892_MASK(6, 0)
#define EN_PUMPX_BIT				BIT(7)

#define CHG_REG_05				0x5
#define ITERM_MASK				BQ25892_MASK(3, 0)

#define CHG_REG_06				0x6
#define VFLOAT_MASK				BQ25892_MASK(7, 2)
#define RECHARGE_VFLOAT_MASK			BIT(0)

#define CHG_REG_07				0x7
#define ITERM_ENABLE				BIT(7)
#define ITERM_DISABLE				0x0
#define ITERM_EN_MASK				BIT(7)
#define STAT_DIS_BIT				BIT(6)
#define STAT_DIS_MASK				BIT(6)
#define I2C_WATCHDOG_DIS_BIT			0x0
#define I2C_WATCHDOG_DIS_MASK			BQ25892_MASK(5, 4)

#define CHG_REG_08				0x8
#define CHG_REG_09				0x9
#define PUMPX_UP_BIT				BIT(1)
#define PUMPX_DN_MASK				BIT(0)
#define PUMPX_DN_DISALBE			0x0

#define CHG_REG_0A				0xa
#define CHG_REG_0B				0xb
#define CHG_REG_0C				0xc
#define CHG_REG_0D				0xd
#define FORCE_VINDPM_DIS_BIT			0x0
#define FORCE_VINDPM_MASK			BIT(7)
#define CHG_REG_0E				0xe
#define BATTERY_VFLOAT_MASK			BQ25892_MASK(6, 0)
#define CHG_REG_0F				0xF
#define CHG_REG_10				0x10
#define CHG_REG_11				0x11
#define CHG_REG_12				0x12

#define CHG_REG_13				0x13
#define IDPM_STAT_BIT				BIT(6)

#define CHG_REG_14				0x14

#define VBUS_VDPM_MIN				4500
#define VBUS_IDPM_MAX				2000

#define ADC_CONVERT_GAP_MIN			(3)

enum {
	NONE_WORK,
	ADCC_WORK,
	CPC_WORK
};

struct async_work_info {
	struct list_head	list;
	int			type;
	union {
		struct {
			int	val;
		} adcc;

		struct {
			int	up;
		} cpc;
	} extra_info;
	int			status;
	void			(*complete)(void *context);
	void			*context;
	int 			timeout;
	struct bq25892_charger 	*chip;
	struct work_struct 	work;
};

static int bq25892_read_reg(struct bq25892_charger *chip, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev, "i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	return 0;
}

static int bq25892_write_reg(struct bq25892_charger *chip, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev, "i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq25892_masked_write(struct bq25892_charger *chip, int reg,
							u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = bq25892_read_reg(chip, reg, &temp);
	if (rc) {
		dev_err(chip->dev, "read failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = bq25892_write_reg(chip, reg, temp);
	if (rc) {
		dev_err(chip->dev, "write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}

static int bq25892_fastchg_current_set(struct bq25892_charger *chip,
                                       unsigned int fastchg_current)
{
	u8 val;

	if ((fastchg_current < BQ25892_CHG_FAST_MIN_MA) ||
	    (fastchg_current > BQ25892_CHG_FAST_MAX_MA)) {
		dev_err(chip->dev, "Invalid pre_fastchg current %d mA requested\n",
		        fastchg_current);
		return -EINVAL;
	}
	val = fastchg_current / 64;

	return bq25892_masked_write(chip, CHG_REG_04, FAST_CHG_CURRENT_MASK, val);
}

#define MIN_FLOAT_MV		3840
#define MAX_FLOAT_MV		4608
#define VFLOAT_STEP_MV		16

static int bq25892_float_voltage_set(struct bq25892_charger *chip,
                                     int vfloat_mv)
{
	u8 temp;
	u8 val;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "invalid float voltage %d mv requested\n", vfloat_mv);
		return -EINVAL;
	}

	val = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;
	temp = (vfloat_mv - MIN_FLOAT_MV) % VFLOAT_STEP_MV;

	if (temp)
		val += 1;

	return bq25892_masked_write(chip, CHG_REG_06, VFLOAT_MASK, val << 2);
}

static int bq25892_iterm_set(struct bq25892_charger *chip, int iterm_ma)
{
	int rc;
	u8 reg;
	int val = 0;

	val = (iterm_ma - 64) / 64;
	if(val >= 0xf)
		reg = 0xf;
	else
		reg = val & 0xf;

	rc = bq25892_masked_write(chip, CHG_REG_05, ITERM_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set iterm rc = %d\n", rc);
		return rc;
	}
	return 0;
}

static int bq25892_get_prop_charge_type(struct bq25892_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = bq25892_read_reg(chip, CHG_REG_0B, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read CHG_REG_0B rc = %d\n", rc);
		return 0;
	}

	if ((reg & 0x18) == 0x01)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	if ((reg & 0x18) == 0x10)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int bq25892_get_prop_batt_status(struct bq25892_charger *chip)
{
	int rc;
	u8 reg = 0;

	if (chip->batt_full)
		return POWER_SUPPLY_STATUS_FULL;

	rc = bq25892_read_reg(chip, CHG_REG_0B, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read CHG_REG_0B rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if ((reg & 0x18) == 0x00) {
		dev_dbg_ratelimited(chip->dev, "%s: Not charging\n", __func__);
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	if ((reg & 0x18) == 0x08 || (reg & 0x18) == 0x10) {
		dev_dbg_ratelimited(chip->dev, "%s: Charging\n", __func__);
		return POWER_SUPPLY_STATUS_CHARGING;
	}
	if ((reg & 0x18) == 0x18) {
		dev_dbg_ratelimited(chip->dev, "%s: Full\n", __func__);
		return POWER_SUPPLY_STATUS_FULL;
	}

	dev_dbg_ratelimited(chip->dev, "%s: Discharging\n", __func__);
	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int bq25892_charging_disable(struct bq25892_charger *chip, int disable)
{
	int rc = 0;

	if (chip->chg_autonomous_mode) {
		dev_info(chip->dev, "Charger in autonomous mode\n");
		return 0;
	}

	rc = bq25892_masked_write(chip, CHG_REG_03, CHG_CONFIG_MASK,
	                          disable ? CHG_CONFIG_DISEN_BIT : CHG_CONFIG_EN_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't disable charging rc=%d\n", rc);

	return rc;
}

static int bq25892_set_idpm_limit(struct bq25892_charger *chip, int idpm_limit)
{
	u8 reg = 0, mask = 0;
	int limit_val;

	if (idpm_limit <= 100)
		idpm_limit = 100;

	limit_val = (idpm_limit - 100) / 50;
	reg = limit_val;
	mask = BQ25892_MASK(5, 0);

	return bq25892_masked_write(chip, CHG_REG_00, mask, reg);
}

static int bq25892_set_usb_chg_current(struct bq25892_charger *chip,
                                       int current_ma)
{
	int rc = 0;
	if (chip->chg_autonomous_mode) {
		dev_info(chip->dev, "Charger in autonomous mode\n");
		return 0;
	}

	rc = bq25892_set_idpm_limit(chip, current_ma);
	if (rc) {
		dev_err(chip->dev, "Couldn't set input mA rc=%d\n", rc);
		return rc;
	}
	return rc;
}

static enum power_supply_property bq25892_parallel_properties[] = {
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

static int bq25892_parallel_set_chg_present(struct bq25892_charger *chip,
						int present)
{
	int rc;
	u8 reg;

	if (present == chip->parallel_charger_present)
		return 0;

	/* Check if BQ25892 is present */
	rc = bq25892_read_reg(chip, CHG_REG_14, &reg);
	if (rc) {
		dev_err(chip->dev, "Failed to detect bq25892-parallel-charger, may be absent\n");
		return -ENODEV;
	}
	chip->parallel_charger_present = present;

	if (!present)
		chip->max_ilim_ma = 0;

	return 0;
}

static bool bq25892_is_input_current_limited(struct bq25892_charger *chip)
{
	int rc;
	u8 reg;

	rc = bq25892_read_reg(chip, CHG_REG_13, &reg);
	if (rc) {
		dev_err(chip->dev, "Failed to read CHG_REG_13 for IINDPM Status: %d\n", rc);
		return false;
	}

	return !!(reg & IDPM_STAT_BIT);
}

static void work_complete(void *arg)
{
	complete(arg);
}

/* ADC conversion */
static int __adcc_work_sync(struct async_work_info *work_info_ptr)
{
	int rc;
	u8 reg = 0;
	unsigned long before;

	rc = bq25892_masked_write(work_info_ptr->chip, CHG_REG_02, CONV_START_MASK, CONV_START_BIT);
	if (rc)
		goto out;
	before = jiffies;
	while (1) {
		/* ADC conversion time is from 8 to 1000 ms */
		msleep(10);
		rc = bq25892_read_reg(work_info_ptr->chip, CHG_REG_02, &reg);
		if (rc)
			break;
		if ((reg & CONV_START_BIT) == 0)
			break;
		if (time_after(jiffies, before + msecs_to_jiffies(work_info_ptr->timeout * 1000))) {
			dev_err(work_info_ptr->chip->dev,
			        "Timeout expired while waiting for ADC conversion.\n");
			rc = -1;
			break;
		}
	}
	dev_dbg(work_info_ptr->chip->dev, "%s took %d ms\n",
	        __func__, jiffies_to_msecs(jiffies - before));
out:
	work_info_ptr->status = rc;
	work_info_ptr->complete(work_info_ptr->context);
	return rc;
}

/* Current pulse control */
static int __cpc_work_sync(struct async_work_info *work_info_ptr)
{
	int rc;
	u8 reg, mask;
	unsigned long before;

	rc = bq25892_masked_write(work_info_ptr->chip, CHG_REG_04, EN_PUMPX_BIT, EN_PUMPX_BIT);
	if (rc)
		goto out;
	if (work_info_ptr->extra_info.cpc.up) {
		reg = PUMPX_UP_BIT;
		mask = PUMPX_UP_BIT;
	} else {
		reg = PUMPX_DN_MASK;
		mask = PUMPX_DN_MASK;
	}
	rc = bq25892_masked_write(work_info_ptr->chip, CHG_REG_09, mask, reg);
	before = jiffies;
	while (1) {
		/* Normally current pulse control plus is about from 70 to 500 msec */
		msleep(100);
		rc = bq25892_read_reg(work_info_ptr->chip, CHG_REG_09, &reg);
		if (rc)
			break;
		if ((reg & mask) == 0)
			break;
		if (time_after(jiffies, before + msecs_to_jiffies(work_info_ptr->timeout * 1000))) {
			dev_err(work_info_ptr->chip->dev,
			        "Timeout expired while waiting for current pulse to finish.\n");
			rc = -1;
			break;
		}
	}
	dev_dbg(work_info_ptr->chip->dev, "%s took %d ms\n",
	        __func__, jiffies_to_msecs(jiffies - before));
out:
	work_info_ptr->status = rc;
	work_info_ptr->complete(work_info_ptr->context);
	return rc;
}

static void bq25892_work_fn(struct work_struct *work)
{
	struct async_work_info *work_info_ptr = container_of(work, struct async_work_info, work);

	switch (work_info_ptr->type) {
	case ADCC_WORK:
		__adcc_work_sync(work_info_ptr);
		break;
	case CPC_WORK:
		__cpc_work_sync(work_info_ptr);
		break;
	default:
		BUG();
		break;
	}
}

static int adcc_work_sync(struct bq25892_charger *chip, int timeout)
{
	struct async_work_info work_info;
	bool b_schedule = false;

	DECLARE_COMPLETION_ONSTACK(done);
	chip->adcc_processing = true;

	mutex_lock(&chip->adcc_lock_mutex);
	work_info.timeout  = timeout;
	work_info.context = &done;
	work_info.complete = work_complete;
	work_info.chip = chip;
	work_info.type = ADCC_WORK;
	work_info.status = -1;
	INIT_WORK_ONSTACK(&work_info.work, bq25892_work_fn);
	b_schedule = schedule_work(&work_info.work);//we use on stack work, so should be ok always!
	mutex_unlock(&chip->adcc_lock_mutex);

	if (b_schedule)
		wait_for_completion(&done);
	if (!work_info.status)
		chip->adcc_last_jiffies = jiffies;
	chip->adcc_processing = false;

	return work_info.status;
}

static int cpc_work_sync(struct bq25892_charger *chip, int timeout, int up)
{
	struct async_work_info work_info;
	bool b_schedule = false;
	DECLARE_COMPLETION_ONSTACK(done);
	chip->cpc_processing = true;

	mutex_lock(&chip->cpc_lock_mutex);
	work_info.timeout  = timeout;
	work_info.context = &done;
	work_info.complete = work_complete;
	work_info.chip = chip;
	work_info.type = CPC_WORK;
	work_info.extra_info.cpc.up = up;
	work_info.status = -1;
	INIT_WORK_ONSTACK(&work_info.work, bq25892_work_fn);
	b_schedule = schedule_work(&work_info.work);//we use on stack work, so should be ok always!
	mutex_unlock(&chip->cpc_lock_mutex);

	if (b_schedule)
		wait_for_completion(&done);
	chip->cpc_processing = false;

	return work_info.status;
}

static int bq25892_adcc_work(struct bq25892_charger *chip, int force)
{
	bool need_launch;
	int  ret = 0;

	need_launch  = time_after(jiffies, chip->adcc_last_jiffies +msecs_to_jiffies(ADC_CONVERT_GAP_MIN * 1000));
	need_launch |= force;
	need_launch &= !chip->adcc_processing;

	if (!need_launch)
		dev_dbg(chip->dev, "%s: no need to launch\n", __func__);
	else
		ret = adcc_work_sync(chip, 1);

	return ret;
}

static int bq25892_cpc_work(struct bq25892_charger *chip, int up)
{
	int ret = 0;

	if (chip->cpc_processing)
		dev_dbg(chip->dev, "%s: no need to launch\n", __func__);
	else
		ret = cpc_work_sync(chip, 5, up);

	return ret;
}

static int bq25892_set_psel(struct bq25892_charger *chip, int supply_type)
{
	int rc = 0;

	if (!gpio_is_valid(chip->psel_gpio))
		return -1;
	if (supply_type != POWER_SUPPLY_TYPE_USB && supply_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		dev_dbg(chip->dev, "Supply type is %d => setting PSEL GPIO low\n",
		        supply_type);
		gpio_set_value(chip->psel_gpio, 0);
	} else {
		dev_dbg(chip->dev, "Supply type is %d => setting PSEL GPIO high\n",
		        supply_type);
		gpio_set_value(chip->psel_gpio, 1);
	}
	return rc;
}

static int bq25892_read_vbus(struct bq25892_charger *chip)
{
	int val;
	u8 reg;
	bq25892_adcc_work(chip, 1 /* force */);
	bq25892_read_reg(chip, CHG_REG_11, &reg);
	val = (reg & 0x7F) * 100 + 2600;
	return val;
}

static int bq25892_set_vdpm_limit(struct bq25892_charger *chip, int vbus_limit){
	u8 reg = 0, mask = 0;
	int limit_val;
	if(vbus_limit < 3900) {
		vbus_limit = 3900;
	}
	limit_val = (vbus_limit - 2600 ) / 100;
	reg = FORCE_VINDPM_MASK | limit_val;
	mask = FORCE_VINDPM_MASK | BQ25892_MASK(6, 0);
	return bq25892_masked_write(chip, CHG_REG_0D, mask, reg);
}

static int  bq25892_is_vdpm_state(struct bq25892_charger *chip)
{
	u8 reg = 0;
	bq25892_adcc_work(chip, 0 /* force */);
	bq25892_read_reg(chip, CHG_REG_13, &reg);
	return !!(reg & 0x80);
}

static const int idpm_fitter[] = {
	//2000,
	1800,
	1500,
	1000
};

static void bq25892_charger_fit(struct bq25892_charger *chip)
{
	int i;
	int fitter_size;
	int val;
	int idpm_limit;

	fitter_size = sizeof(idpm_fitter) / sizeof(idpm_fitter[0]);
	for (i = 0; i < fitter_size; i++) {
		idpm_limit = idpm_fitter[i];
		bq25892_set_idpm_limit(chip, idpm_limit);
		bq25892_cpc_work(chip, 0);
		val = bq25892_read_vbus(chip);
		dev_info(chip->dev, "%s: vbus is %d\n", __func__, val);
		if (val <= VBUS_VDPM_MIN || bq25892_is_vdpm_state(chip))
			continue;
		else
			break;
	}
	if (idpm_limit == 1800) {
		idpm_limit = 2000;
		bq25892_set_idpm_limit(chip, idpm_limit);
	}
	chip->max_ilim_ma = idpm_limit;
}

static int bq25892_mtk_handshake(struct bq25892_charger *chip, int target_voltage_mv)
{
	int val;
	int prev_val;
	bool up;
	int rc = -1;
	int need_cpc_down = 0;

	/* Before launching the handshake, we should set idpm_limit
	 * to 1A, then after done, recover the idpm_limit.
	 */
	bq25892_set_idpm_limit(chip, 500);
	val = bq25892_read_vbus(chip);
	dev_info(chip->dev, "%s: vbus %d mv, raise voltage to %d mv\n",
	         __func__, val, target_voltage_mv);
	if (val > target_voltage_mv + 1000)
		up = false;
	else if ((val + 1000) < target_voltage_mv)
		up = true;
	else
		goto done; /* skip */

	dev_dbg(chip->dev, "%s: current vbus = %d, decision: DOWN\n",
	        __func__, val);
	if (up) {
		do {
			prev_val = bq25892_read_vbus(chip);
			bq25892_cpc_work(chip, 1);
			val = bq25892_read_vbus(chip);
			dev_dbg(chip->dev, "%s: cpc prev_vbus %d mv, vbus %d mv\n",
			        __func__, prev_val, val);
			if (val < (prev_val + 1000)) {
				dev_dbg(chip->dev, "%s: vbus not high enough, "
				        "checking again after delay\n", __func__);
				msleep(250);
				val = bq25892_read_vbus(chip);
			}

			if (val > (prev_val + 1000)) {
				if (val > (target_voltage_mv  + 1000)) {
					need_cpc_down = 1;
					break;
				} else if ((val + 1000) > target_voltage_mv) {
					rc = 0;
					break;
				}
			} else
				break;
		} while(1);
		if (need_cpc_down)
			bq25892_cpc_work(chip, 0);
	} else {
		do {
			prev_val = bq25892_read_vbus(chip);
			bq25892_cpc_work(chip, 0);
			val = bq25892_read_vbus(chip);
			dev_dbg(chip->dev, "%s: cpc prev_vbus %d mv, vbus %d mv\n",
			        __func__, prev_val, val);
			if (prev_val < (val + 1000)) {
				dev_dbg(chip->dev, "%s: vbus not high enough, "
				        "checking again after delay\n", __func__);
				msleep(250);
				val = bq25892_read_vbus(chip);
			}

			if ((val + 1000) < prev_val) {
				if (val <= (target_voltage_mv + 1000)) {
					rc = 0;
					break;
				}
			} else
				break;
		} while(1);
	}
done:
	bq25892_set_idpm_limit(chip, chip->usb_psy_ma);
	dev_dbg(chip->dev, "%s: done\n", __func__);
	return rc;
}

static char *bq25892_wake_reason_to_string(enum wake_reason_to_parallel reason)
{
	switch (reason) {
	case PM_TO_PARALLEL_PARALLEL_CHECK:
		return "PM_TO_PARALLEL_PARALLEL_CHECK";
	case PM_TO_PARALLEL_TIMER_SOC:
		return "PM_TO_PARALLEL_TIMER_SOC";
	default:
		return "(unknown)";
	}
}

static void bq25892_stay_awake(struct bq25892_charger *chip,
                               enum wake_reason_to_parallel reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons | reason;
	if (reasons != 0 && chip->wake_reasons == 0) {
		dev_dbg(chip->dev, "staying awake: reason %s (mask 0x%02x)\n",
		        bq25892_wake_reason_to_string(reason), reasons);
		pm_stay_awake(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
}

static void bq25892_relax(struct bq25892_charger *chip,
                          enum wake_reason_to_parallel reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons & (~reason);
	if (reasons == 0 && chip->wake_reasons != 0) {
		dev_dbg(chip->dev, "relaxing: reason %s (mask 0x%02x)\n",
		        bq25892_wake_reason_to_string(reason), reasons);
		pm_relax(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
};

static int bq25892_parallel_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct bq25892_charger *chip = container_of(psy,
				struct bq25892_charger, parallel_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			rc = bq25892_charging_disable(chip, !val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		rc = bq25892_parallel_set_chg_present(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (chip->fastchg_current_max_ma != val->intval / 1000) {
			chip->fastchg_current_max_ma = val->intval / 1000;
			rc = bq25892_fastchg_current_set(chip,
					chip->fastchg_current_max_ma);
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (chip->usb_psy_ma != val->intval / 1000) {
			chip->usb_psy_ma = val->intval / 1000;
			rc = bq25892_set_usb_chg_current(chip,
						chip->usb_psy_ma);
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (chip->parallel_charger_present &&
			(chip->vfloat_mv != val->intval)) {
			rc = bq25892_float_voltage_set(chip, val->intval);
			if (!rc)
				chip->vfloat_mv = val->intval;
		} else {
			chip->vfloat_mv = val->intval;
		}
		break;
	case POWER_SUPPLY_PROP_MTK_HANDSHAKE:
		if (chip->parallel_charger_present) {
			rc = bq25892_mtk_handshake(chip, val->intval * 1000);
		} else {
			rc = -1;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_FIT:
		bq25892_charger_fit(chip);
		break;
	case POWER_SUPPLY_PROP_USB_HC:
		rc = bq25892_set_psel(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_PM_CONTROL_WAKE:
		bq25892_stay_awake(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_PM_CONTROL_RELAX:
		bq25892_relax(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}
	return rc;
}

static int bq25892_parallel_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		return 1;
	default:
		return 0;
	}
}

static int bq25892_parallel_batt_voltage_now_mv(struct bq25892_charger *chip)
{
	u8  reg;

	bq25892_adcc_work(chip, 0 /* force */);
	bq25892_read_reg(chip, CHG_REG_0E, &reg);

	return (reg & 0x7F) * 20 + 2304;
}

static int bq25892_parallel_input_current_base_ico(struct bq25892_charger *chip)
{
	int rc = 0, i = 0, sum = 0;
	u8 reg;
	int val = 0;
	/* read ICO */
	rc = bq25892_read_reg(chip, CHG_REG_02, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set PUMPX_UP pin rc=%d\n", rc);
		return false;
	}
	if (reg & 0x10) {
		dev_dbg(chip->dev, "ico is enabled\n");
		/* read IDPM_LIM */
		rc = bq25892_read_reg(chip, CHG_REG_13, &reg);
		if (rc) {
			dev_err(chip->dev, "Failed to read IDPM_LIM: %d\n", rc);
			return false;
		}
	} else {
		dev_dbg(chip->dev, "ico is disabled\n");
		/* read IINLIM */
		rc = bq25892_read_reg(chip, CHG_REG_00, &reg);
		if (rc) {
			dev_err(chip->dev, "Failed to read IINLIM: %d\n", rc);
			return false;
		}
	}
	reg &= 0x3F;
	while (reg) {
		val = reg % 2;
		sum = sum + val * 50 * (1 << i);
		i++;
		reg = reg / 2;
	}
	sum += 100;
	dev_dbg(chip->dev, "%s: sum = %d\n", __func__, sum);
	return sum;
}


static int bq25892_parallel_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct bq25892_charger *chip = container_of(psy,
				struct bq25892_charger, parallel_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !chip->charging_disabled_status;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (chip->parallel_charger_present)
			val->intval = chip->usb_psy_ma * 1000;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = chip->vfloat_mv * 1000; /* mV to uV */
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->parallel_charger_present;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->fastchg_current_max_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (chip->parallel_charger_present)
			val->intval = bq25892_get_prop_batt_status(chip);
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		val->intval =
			bq25892_is_input_current_limited(chip) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = chip->max_ilim_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_NOW:
		if (chip->parallel_charger_present)
			val->intval = bq25892_parallel_input_current_base_ico(chip) * 1000;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval  = bq25892_parallel_batt_voltage_now_mv(chip);
		val->intval *= 1000; /* mV to uV */
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq25892_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_VBUS_NOW:
		val->intval = bq25892_read_vbus(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		{
			u8 reg;
			bq25892_adcc_work(chip, 0 /* force */);
			bq25892_read_reg(chip, CHG_REG_12, &reg);
			val->intval = (reg & 0x7F) * 50;
		}
		break;
	case POWER_SUPPLY_PROP_IINDPM_STATUS:
		{
			u8 reg;
			bq25892_adcc_work(chip, 0 /* force */);
			bq25892_read_reg(chip, CHG_REG_13, &reg);
			val->intval = !!(reg & 0x40);
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2
static irqreturn_t bq25892_chg_stat_handler(int irq, void *dev_id)
{
	struct bq25892_charger *chip = dev_id;
	u8 rt_stat, prev_rt_stat;
	int handler_count = 0;
	int rc;

	mutex_lock(&chip->irq_complete);

	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		dev_warn(chip->dev, "IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	rc = bq25892_read_reg(chip, CHG_REG_0C, &prev_rt_stat);
	if (rc)
		dev_dbg(chip->dev, "%s: Couldn't read initial CHG_REG_0C, rc = %d\n",
		        __func__, rc);
	else
		dev_dbg(chip->dev, "%s: First CHG_REG_0C = %x\n",
		        __func__, prev_rt_stat);

	bq25892_read_reg(chip, CHG_REG_0C, &rt_stat);
	if (rc)
		dev_err(chip->dev, "Couldn't read second time CHG_REG_0C rc = %d\n", rc);
	else
		dev_dbg(chip->dev, "%s:, Second CHG_REG_0C = %x\n", __func__, rt_stat);

	/* FIXME: Shouldn't the handler_count increase? Shouldn't we do anything with rt_stat? */
	dev_info(chip->dev, "handler count = %d\n", handler_count);
	if (handler_count) {
		dev_info(chip->dev, "batt psy changed\n");
		power_supply_changed(&chip->parallel_psy);
	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static void async_work_init(struct bq25892_charger *chip)
{
	mutex_init(&chip->adcc_lock_mutex);
	mutex_init(&chip->cpc_lock_mutex);
	mutex_init(&chip->pm_lock);
	chip->adcc_processing = false;
	chip->cpc_processing = false;
}

static int get_reg(void *data, u64 *val)
{
	struct bq25892_charger *chip = data;
	int rc;
	u8 temp;

	rc = bq25892_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc) {
		dev_err(chip->dev, "Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct bq25892_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = bq25892_write_reg(chip, chip->peek_poke_address, temp);
	if (rc) {
		dev_err(chip->dev, "Couldn't write 0x%02x to 0x%02x rc= %d\n",
			temp, chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int force_irq_set(void *data, u64 val)
{
	struct bq25892_charger *chip = data;

	bq25892_chg_stat_handler(chip->client->irq, data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_irq_ops, NULL, force_irq_set, "0x%02llx\n");

static int bq25892_parse_dt(struct bq25892_charger *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	chip->charging_disabled_status = of_property_read_bool(node,
		"qcom,charging-disabled");

	chip->chg_autonomous_mode = of_property_read_bool(node,
		"qcom,chg-autonomous-mode");

	chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");

	chip->using_pmic_therm = of_property_read_bool(node,
		"qcom,using-pmic-therm");
	chip->bms_controlled_charging  = of_property_read_bool(node,
		"qcom,bms-controlled-charging");
	chip->force_hvdcp_2p0 = of_property_read_bool(node,
		"qcom,force-hvdcp-2p0");

	rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma",
	                          &chip->fastchg_current_max_ma);
	if (rc)
		chip->fastchg_current_max_ma = BQ25892_CHG_FAST_MAX_MA;

	chip->iterm_disabled = of_property_read_bool(node,
		"qcom,iterm-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc)
		chip->iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
	                          &chip->vfloat_mv);
	if (rc)
		chip->vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,recharge-mv",
	                          &chip->recharge_mv);
	if (rc)
		chip->recharge_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node,
		"qcom,recharge-disabled");

	/* thermal and jeita support */
	rc = of_property_read_u32(node, "qcom,batt-cold-decidegc",
		&chip->batt_cold_decidegc);
	if (rc < 0)
		chip->batt_cold_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,batt-hot-decidegc",
		&chip->batt_hot_decidegc);
	if (rc < 0)
		chip->batt_hot_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,batt-warm-decidegc",
		&chip->batt_warm_decidegc);

	rc |= of_property_read_u32(node, "qcom,batt-cool-decidegc",
		&chip->batt_cool_decidegc);

	if (!rc) {
		rc = of_property_read_u32(node, "qcom,batt-cool-mv",
		                          &chip->batt_cool_mv);

		rc |= of_property_read_u32(node, "qcom,batt-warm-mv",
		                           &chip->batt_warm_mv);

		rc |= of_property_read_u32(node, "qcom,batt-cool-ma",
		                           &chip->batt_cool_ma);

		rc |= of_property_read_u32(node, "qcom,batt-warm-ma",
		                           &chip->batt_warm_ma);
		if (rc)
			chip->jeita_supported = false;
		else
			chip->jeita_supported = true;
	}

	dev_info(chip->dev, "jeita_supported = %d\n", chip->jeita_supported);

	rc = of_property_read_u32(node, "qcom,batt-missing-decidegc",
	                          &chip->batt_missing_decidegc);

	rc = of_property_read_u32(node, "qcom,parallel-en-pin-polarity",
	                          &chip->parallel_pin_polarity_setting);
	if (rc)
		chip->parallel_pin_polarity_setting = EN_BY_PIN_LOW_ENABLE;
	else
		chip->parallel_pin_polarity_setting =
				chip->parallel_pin_polarity_setting ?
				EN_BY_PIN_HIGH_ENABLE : EN_BY_PIN_LOW_ENABLE;

	chip->psel_gpio = of_get_named_gpio(node, "qcom,psel-gpio", 0);
	rc = gpio_request(chip->psel_gpio, "psel_gpio");
	if (rc)
		dev_err(chip->dev, "request reset gpio failed, rc=%d\n", rc);

	return 0;
}

static int bq25892_hw_init(struct bq25892_charger *chip)
{
	int rc;
	u8 reg = 0, mask = 0;

	/*
	 * If the charger is pre-configured for autonomous operation,
	 * do not apply additional settings
	 */
	if (chip->chg_autonomous_mode) {
		dev_info(chip->dev, "Charger configured for autonomous mode\n");
		return 0;
	}

	/* Disable state pin and I2C Watchdog Timer */
	reg = STAT_DIS_BIT | I2C_WATCHDOG_DIS_BIT;
	mask = STAT_DIS_MASK | I2C_WATCHDOG_DIS_MASK;
	rc = bq25892_masked_write(chip, CHG_REG_07, mask, reg);

	/*set WD_RST*/
	rc = bq25892_masked_write(chip, CHG_REG_03, WD_RST_BIT, WD_RST_BIT);

	/*read reg0c*/
	rc = bq25892_read_reg(chip, CHG_REG_0C, &reg);
	dev_info(chip->dev, "CHG_REG_0C = %x\n", reg);

	/*disable ILIM Pin*/
	rc = bq25892_masked_write(chip, CHG_REG_00, EN_ILIM_BIT, 0x0);
	if (rc)
		dev_err(chip->dev, "Unable to disable charging. rc=%d\n", rc);

	/* disable charging, and pmi8952 will enable it */
	rc = bq25892_masked_write(chip, CHG_REG_03, CHG_CONFIG_MASK, CHG_CONFIG_DISEN_BIT);
	if (rc)
		dev_err(chip->dev, "Unable to disable charging. rc=%d\n", rc);

	/*IR compensation for 40m ou in order to speed up the charging cycle, we known current is 24m ou*/
	reg = BIT(6) | BIT(4) | BIT(2);
	mask = BQ25892_MASK(7, 5) | BQ25892_MASK(4, 2);
	rc = bq25892_masked_write(chip, CHG_REG_08, mask, reg);

	/*setup handshake*/
	reg = AUTO_DPDM_EN_BIT | MAXC_EN_BIT | HVDCP_EN_BIT |ICO_EN_BIT;
	mask = AUTO_DPDM_EN_MASK | MAXC_EN_MASK | HVDCP_EN_MASK | ICO_EN_MASK;
	rc = bq25892_masked_write(chip, CHG_REG_02, mask, reg);
	/* set iterm */
	if (chip->iterm_ma != -EINVAL) {
		rc = bq25892_iterm_set(chip, chip->iterm_ma);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	if (chip->iterm_disabled) {
		rc = bq25892_masked_write(chip, CHG_REG_07, ITERM_EN_MASK, ITERM_DISABLE);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = bq25892_float_voltage_set(chip, chip->vfloat_mv);
		if (rc) {
			dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

	/* set recharge-threshold */
	if (chip->recharge_mv != -EINVAL) {
		if(chip->recharge_mv <= 100)
			reg = 0x0;
		else
			reg = 0x1;
		rc = bq25892_masked_write(chip, CHG_REG_06,
				RECHARGE_VFLOAT_MASK, reg );
		if (rc) {
			dev_err(chip->dev, "Couldn't set recharge_mv rc = %d\n", rc);
			return rc;
		}
	}


	/* set the fast charge current limit */
	rc = bq25892_fastchg_current_set(chip, chip->fastchg_current_max_ma);
	if (rc) {
		dev_err(chip->dev, "Couldn't set fastchg current rc=%d\n", rc);
		return rc;
	}

	/*set Input Voltage Limit Threshold */
	reg = BQ25892_MASK(7, 6);
	mask = BHOT_MASK;

	rc = bq25892_masked_write(chip, CHG_REG_01, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set FORCE_VINDPM rc = %d\n", rc);
		return rc;
	}
	rc = bq25892_read_reg(chip, CHG_REG_01, &reg);
	dev_info(chip->dev, "CHG_REG_01 = %x\n", reg);

	rc = bq25892_set_vdpm_limit(chip, VBUS_VDPM_MIN);
	if (rc) {
		dev_err(chip->dev, "Couldn't set FORCE_VINDPM rc = %d\n", rc);
		return rc;
	}

	if (gpio_is_valid(chip->psel_gpio)) {
		/*set psel_gpio default high*/
		rc = gpio_direction_output(chip->psel_gpio, 1);
		gpio_set_value(chip->psel_gpio, 1);
	} else {
		dev_info(chip->dev, "%s: psel_gpio invalid\n", __func__);
	}

	chip->usb_psy_ma = DEFAULT_INPUT_CURRENT;
	bq25892_set_usb_chg_current(chip, DEFAULT_INPUT_CURRENT);

	return rc;
}

static int bq25892_parallel_charger_probe(struct i2c_client *client,
                                          const struct i2c_device_id *id)
{
	int rc;
	struct bq25892_charger *chip;
	u8 reg;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;

	async_work_init(chip);
	/* probe the device to check if its actually connected */
	rc = bq25892_read_reg(chip, CHG_REG_14, &reg);
	if (rc) {
		dev_err(chip->dev, "Failed to detect bq25892, device may be absent\n");
		return -ENODEV;
	}
	dev_info(chip->dev, "bq25892 chip revision is %x\n", reg & 0x3);

	rc = bq25892_parse_dt(chip);
	if (rc) {
		dev_err(chip->dev, "Couldn't parse DT nodes rc=%d\n", rc);
		return rc;
	}

	i2c_set_clientdata(client, chip);

	rc = of_property_read_string(chip->dev->of_node,
	                             "qcom,parallel-psy-name",
	                             &chip->parallel_psy.name);
	if (rc)
		chip->parallel_psy.name = "usb-parallel";

	chip->parallel_psy.type		= POWER_SUPPLY_TYPE_USB_PARALLEL;
	chip->parallel_psy.get_property	= bq25892_parallel_get_property;
	chip->parallel_psy.set_property	= bq25892_parallel_set_property;
	chip->parallel_psy.properties	= bq25892_parallel_properties;
	chip->parallel_psy.property_is_writeable
				= bq25892_parallel_is_writeable;
	chip->parallel_psy.num_properties
				= ARRAY_SIZE(bq25892_parallel_properties);

	rc = power_supply_register(chip->dev, &chip->parallel_psy);
	if (rc) {
		dev_err(chip->dev, "Couldn't register parallel psy rc=%d\n", rc);
		return rc;
	}

	chip->resume_completed = true;
	mutex_init(&chip->irq_complete);
	rc = bq25892_hw_init(chip);
	if (rc) {
		dev_err(chip->dev, "Couldn't intialize hardware rc=%d\n", rc);
		goto fail_bq25892_regulator_init;
	}

	/* STAT irq configuration */
	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				bq25892_chg_stat_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq25892_chg_stat_irq", chip);
		if (rc) {
			dev_err(chip->dev, "Failed STAT irq=%d request rc = %d\n",
				client->irq, rc);
			goto fail_bq25892_regulator_init;
		}
		enable_irq_wake(client->irq);
	}

	dev_info(chip->dev, "bq25892 parallel successfully probed.\n");
	return 0;

fail_bq25892_regulator_init:
	power_supply_unregister(&chip->parallel_psy);
	return rc;
}

static int bq25892_charger_remove(struct i2c_client *client)
{
	struct bq25892_charger *chip = i2c_get_clientdata(client);

	gpio_set_value(chip->psel_gpio, 1);
	gpio_free(chip->psel_gpio);
	cancel_delayed_work_sync(&chip->chg_remove_work);

	mutex_destroy(&chip->irq_complete);
	debugfs_remove_recursive(chip->debug_root);
	return 0;
}

static struct of_device_id bq25892_match_table[] = {
	{ .compatible = "qcom,bq25892-charger",},
	{ },
};

static const struct i2c_device_id bq25892_charger_id[] = {
	{"bq25892-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25892_charger_id);

static struct i2c_driver bq25892_charger_driver = {
	.driver		= {
		.name		= "bq25892-charger",
		.owner		= THIS_MODULE,
		.of_match_table	= bq25892_match_table,
	},
	.probe		= bq25892_parallel_charger_probe,
	.remove		= bq25892_charger_remove,
	.id_table	= bq25892_charger_id,
};

module_i2c_driver(bq25892_charger_driver);

MODULE_DESCRIPTION("bq25892 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:bq25892-charger");
