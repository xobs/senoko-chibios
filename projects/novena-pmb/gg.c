#include <ch.h>
#include <hal.h>
#include <i2c.h>

#include <string.h>

#include "pmb.h"

#define GG_ADDR 0xb

enum power_supply_property {
	/* Properties of type `int' */
	POWER_SUPPLY_PROP_STATUS = 0,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_AUTHENTIC,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_AVG,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_ENERGY_AVG,
	POWER_SUPPLY_PROP_CAPACITY, /* in percents! */
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN, /* in percents! */
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX, /* in percents! */
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
	POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MAX,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TYPE, /* use power_supply.type instead */
	POWER_SUPPLY_PROP_SCOPE,
	/* Properties of type `const char *' */
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

enum {
	REG_MANUFACTURER_DATA,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_CAPACITY,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CYCLE_COUNT,
	REG_SERIAL_NUMBER,
	REG_REMAINING_CAPACITY,
	REG_REMAINING_CAPACITY_CHARGE,
	REG_FULL_CHARGE_CAPACITY,
	REG_FULL_CHARGE_CAPACITY_CHARGE,
	REG_DESIGN_CAPACITY,
	REG_DESIGN_CAPACITY_CHARGE,
	REG_DESIGN_VOLTAGE,
};

/* Battery Mode defines */
#define BATTERY_MODE_OFFSET             0x03
#define BATTERY_MODE_MASK               0x8000
enum sbs_battery_mode {
        BATTERY_MODE_AMPS,
        BATTERY_MODE_WATTS
};

/* manufacturer access defines */
#define MANUFACTURER_ACCESS_STATUS      0x0006
#define MANUFACTURER_ACCESS_SLEEP       0x0011

/* battery status value bits */
#define BATTERY_DISCHARGING             0x40
#define BATTERY_FULL_CHARGED            0x20
#define BATTERY_FULL_DISCHARGED         0x10

#define SBS_DATA(_psp, _addr, _min_value, _max_value) { \
	.psp = _psp, \
	.addr = _addr, \
	.min_value = _min_value, \
	.max_value = _max_value, \
}

static const struct chip_data {
	enum power_supply_property psp;
	uint8_t addr;
	int min_value;
	int max_value;
} sbs_data[] = {
	[REG_MANUFACTURER_DATA] =
		SBS_DATA(POWER_SUPPLY_PROP_PRESENT, 0x00, 0, 65535),
	[REG_TEMPERATURE] =
		SBS_DATA(POWER_SUPPLY_PROP_TEMP, 0x08, 0, 65535),
	[REG_VOLTAGE] =
		SBS_DATA(POWER_SUPPLY_PROP_VOLTAGE_NOW, 0x09, 0, 20000),
	[REG_CURRENT] =
		SBS_DATA(POWER_SUPPLY_PROP_CURRENT_NOW, 0x0A, -32768, 32767),
	[REG_CAPACITY] =
		SBS_DATA(POWER_SUPPLY_PROP_CAPACITY, 0x0D, 0, 100),
	[REG_REMAINING_CAPACITY] =
		SBS_DATA(POWER_SUPPLY_PROP_ENERGY_NOW, 0x0F, 0, 65535),
	[REG_REMAINING_CAPACITY_CHARGE] =
		SBS_DATA(POWER_SUPPLY_PROP_CHARGE_NOW, 0x0F, 0, 65535),
	[REG_FULL_CHARGE_CAPACITY] =
		SBS_DATA(POWER_SUPPLY_PROP_ENERGY_FULL, 0x10, 0, 65535),
	[REG_FULL_CHARGE_CAPACITY_CHARGE] =
		SBS_DATA(POWER_SUPPLY_PROP_CHARGE_FULL, 0x10, 0, 65535),
	[REG_TIME_TO_EMPTY] =
		SBS_DATA(POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG, 0x12, 0, 65535),
	[REG_TIME_TO_FULL] =
		SBS_DATA(POWER_SUPPLY_PROP_TIME_TO_FULL_AVG, 0x13, 0, 65535),
	[REG_STATUS] =
		SBS_DATA(POWER_SUPPLY_PROP_STATUS, 0x16, 0, 65535),
	[REG_CYCLE_COUNT] =
		SBS_DATA(POWER_SUPPLY_PROP_CYCLE_COUNT, 0x17, 0, 65535),
	[REG_DESIGN_CAPACITY] =
		SBS_DATA(POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN, 0x18, 0, 65535),
	[REG_DESIGN_CAPACITY_CHARGE] =
		SBS_DATA(POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, 0x18, 0, 65535),
	[REG_DESIGN_VOLTAGE] =
		SBS_DATA(POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, 0x19, 0, 65535),
	[REG_SERIAL_NUMBER] =
		SBS_DATA(POWER_SUPPLY_PROP_SERIAL_NUMBER, 0x1C, 0, 65535),
};



struct gg_string {
	uint8_t size;
	uint8_t reg;
};

static const struct gg_string gg_strings[] = {
	{
		.size = 12,
		.reg  = 0x20,
	},
	{
		.size = 8,
		.reg  = 0x21,
	},
	{
		.size = 5,
		.reg  = 0x22,
	},
};


static int gg_getblock_real(struct I2CDriver *driver, uint8_t reg, void *data, int size) {
	msg_t status    = RDY_OK;
	systime_t tmo   = TIME_INFINITE; /* SMBus times out after 25ms in hardware */
	i2caddr_t addr  = GG_ADDR;

	pmb_smbus_init(driver);
	i2cAcquireBus(driver);
	status = i2cMasterTransmitTimeout(driver, addr,
					&reg, sizeof(reg),
					data, size,
					tmo);
	i2cReleaseBus(driver);
	pmb_smbus_deinit(driver);

	if (status != RDY_OK) {
		if (status == RDY_TIMEOUT)
			return -1;
		return i2cGetErrors(driver) | 0x80000000;
	}

	return size;
}

static int gg_getblock(struct I2CDriver *driver, uint8_t reg, void *data, int size) {
	int tries = 50;
	int ret = 0;
	while (tries-- > 0) {
		ret = gg_getblock_real(driver, reg, data, size);
		if (ret > 0)
			return ret;
	}
	return ret;
}

static int gg_getword(struct I2CDriver *driver, uint8_t reg, void *word) {
	return gg_getblock(driver, reg, word, 2);
}

static int gg_getbyte(struct I2CDriver *driver, uint8_t reg, void *byte) {
	return gg_getblock(driver, reg, byte, 1);
}

static int gg_getstring(struct I2CDriver *driver, uint8_t addr, uint8_t *data, int size) {
	int ret;
	int i;

	ret = gg_getblock(driver, addr, data, size);
	if (ret < 0)
		return ret;

	size--;
	if (data[0] < size)
		size = data[0];
	for (i=0; i<size; i++)
		data[i] = data[i+1];
	data[i] = '\0';
	return size;
}

int gg_init(struct I2CDriver *driver) {
	(void)driver;
	/* Pull GG_SYSPRES low to bring it out of reset */
	palWritePad(GPIOA, PA11, 0);

	return 0;
}

/**
 *
 */
int gg_refresh(struct I2CDriver *driver, int property) {
	uint16_t result = 0;
	uint16_t addr   = property;
	msg_t status    = RDY_OK;
	systime_t tmo   = MS2ST(4);

	i2cAcquireBus(driver);
	status = i2cMasterTransmitTimeout(driver, GG_ADDR,
					(void *)&addr, sizeof(addr),
					(void *)&result, sizeof(result),
					tmo);
	i2cReleaseBus(driver);

	if (status != RDY_OK){
		return i2cGetErrors(driver) | 0x80000000;
	}

	return result & 0x0000ffff;
}

int gg_manuf(struct I2CDriver *driver, uint8_t *manuf) {
	return gg_getstring(driver, gg_strings[0].reg, manuf, gg_strings[0].size);
}

int gg_partname(struct I2CDriver *driver, uint8_t name[8]) {
	return gg_getstring(driver, gg_strings[1].reg, name, gg_strings[1].size);
}

int gg_chem(struct I2CDriver *driver, uint8_t *chem) {
	return gg_getstring(driver, gg_strings[2].reg, chem, gg_strings[2].size);
}

int gg_serial(struct I2CDriver *driver, void *serial) {
	return gg_getword(driver, sbs_data[REG_SERIAL_NUMBER].addr, serial);
}

int gg_percent(struct I2CDriver *driver, void *capacity) {
	return gg_getbyte(driver, sbs_data[REG_CAPACITY].addr, capacity);
}

int gg_cellvoltage(struct I2CDriver *driver, int cell, void *voltage) {
	if (cell <= 0 || cell > 4)
		return -1;
	cell--;
	cell = 3-cell;
	return gg_getword(driver, 0x3c+cell, voltage);
}

