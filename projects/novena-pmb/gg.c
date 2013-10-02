#include <ch.h>
#include <hal.h>
#include <i2c.h>

#include <string.h>

#include "gg.h"
#include "pmb.h"
#include "bionic.h"

#define GG_ADDR 0xb

#define STREAM_SERIAL   (&SD1)
#define STREAM          ((BaseSequentialStream *)STREAM_SERIAL)

/* SMBus times out after 25ms in hardware */
static systime_t tmo   = TIME_INFINITE;

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

static struct cell_cfg {
	uint16_t pov_threshold;
	uint16_t pov_recovery;
	uint16_t puv_threshold;
	uint16_t puv_recovery;
	uint16_t sov_threshold;
	uint16_t charging_voltage;
	uint16_t depleted_voltage;
	uint16_t depleted_recovery;
	uint16_t design_voltage;
	uint16_t flash_update_ok_voltage;
	uint16_t shutdown_voltage;
	uint16_t term_voltage;
} cell_cfgs[] = {
	[2] = {
		.pov_threshold = 8700,
		.pov_recovery = 8400,
		.puv_threshold = 5400,
		.puv_recovery = 5700,
		.sov_threshold = 9000,
		.charging_voltage = 8400,
		.depleted_voltage = 5000,
		.depleted_recovery = 5500,
		.design_voltage = 7200,
		.flash_update_ok_voltage = 6000,
		.shutdown_voltage = 5000,
		.term_voltage = 6000,
	},
	[3] = {
		.pov_threshold = 13000,
		.pov_recovery = 12600,
		.puv_threshold = 8100,
		.puv_recovery = 8500,
		.sov_threshold = 13500,
		.charging_voltage = 12600,
		.depleted_voltage = 8000,
		.depleted_recovery = 8500,
		.design_voltage = 10800,
		.flash_update_ok_voltage = 7500,
		.shutdown_voltage = 7000,
		.term_voltage = 9000,
	},
	[4] = {
		.pov_threshold = 17500,
		.pov_recovery = 16000,
		.puv_threshold = 11000,
		.puv_recovery = 12000,
		.sov_threshold = 18000,
		.charging_voltage = 16800,
		.depleted_voltage = 11000,
		.depleted_recovery = 11500,
		.design_voltage = 14400,
		.flash_update_ok_voltage = 7500,
		.shutdown_voltage = 7000,
		.term_voltage = 12000,
	},
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


static uint8_t subclass_size[] = {
	[0] = 22,
	[1] = 25,
	[2] = 10,
	[3] = 1,

	[16] = 12,
	[17] = 6,
	[18] = 9,
	[19] = 3,
	[20] = 5,
	[21] = 3,

	[32] = 6,
	[33] = 8,
	[34] = 10,
	[35] = 8,
	[36] = 13,
	[37] = 2,
	[38] = 22,

	[48] = 51,
	[49] = 14,

	[56] = 10,

	[58] = 32,
	[59] = 30,
	[60] = 4,

	[64] = 10,

	[67] = 20,
	[68] = 20,

	[80] = 68,
	[81] = 8,
	[82] = 27,
	[88] = 32,
	[89] = 32,
	[90] = 32,
	[91] = 32,
	[92] = 32,
	[93] = 32,
	[94] = 32,
	[95] = 32,
	[96] = 30,
	[97] = 9,

	[104] = 21,
	[105] = 19,
	[106] = 24,
	[107] = 3,
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


/*
 * HACK: If we've just transmitted one byte, then the next I2C transfer
 * will hang.  Reset DMA to prevent this from happening.
 */

static int hack_if_necessary(struct I2CDriver *driver, int size) {
	if (size == 1) {
		pmb_smbus_deinit(driver);
		pmb_smbus_init(driver);
		chThdSleepMilliseconds(5);
	}
	return 0;
}

int gg_init(struct I2CDriver *driver) {
	(void)driver;
	/* Pull GG_SYSPRES low to bring it out of reset */
	palWritePad(GPIOA, PA11, 0);
	return 0;
}

static int gg_getmfgr(struct I2CDriver *driver, uint16_t reg,
		void *data, int size)
{
	msg_t status;
	uint8_t bfr[3];

	bfr[0] = 0;
	bfr[1] = reg;
	bfr[2] = reg>>8;

	i2cAcquireBus(driver);
	status = i2cMasterTransmitTimeout(driver, GG_ADDR,
					bfr, 3,
					NULL, 0,
					tmo);
	if (data && size) {
		status = i2cMasterTransmitTimeout(driver, GG_ADDR,
						bfr, 1,
						data, size,
						tmo);
		hack_if_necessary(driver, size);
	}
	i2cReleaseBus(driver);

	if (status != RDY_OK) {
		if (status == RDY_TIMEOUT)
			return -1;
		return i2cGetErrors(driver) | 0x80000000;
	}
	return 0;
}

static int gg_getblock(struct I2CDriver *driver, uint8_t reg,
		void *data, int size)
{
	msg_t status;

	i2cAcquireBus(driver);
	status = i2cMasterTransmitTimeout(driver, GG_ADDR,
					&reg, sizeof(reg),
					data, size,
					tmo);
	i2cReleaseBus(driver);
	hack_if_necessary(driver, size);

	if (status != RDY_OK) {
		if (status == RDY_TIMEOUT)
			return -1;
		return i2cGetErrors(driver) | 0x80000000;
	}
	return 0;
}

int gg_getdataflash(struct I2CDriver *driver,
		uint8_t subclass, uint8_t offset, void *data, int size) {
	msg_t status;
	uint8_t bfr[3];
	uint8_t *cdata = data;
	uint8_t reg;
	int ptr;

	for (ptr=0; ptr<size; ptr++)
		cdata[ptr] = 0;

	i2cAcquireBus(driver);

	bfr[0] = 0x77; /* SetSubclassID register */
	bfr[1] = subclass;
	bfr[2] = subclass>>8;


	status = i2cMasterTransmitTimeout(driver, GG_ADDR,
					bfr, sizeof(bfr),
					NULL, 0,
					tmo);
	if (status != RDY_OK)
		goto err;

	reg = (offset/32)+0x78;
	while (size > 0) {
		/*
		 * If we're starting at a non-divisible offset,
		 * copy the data into an intermediate buffer first.
		 */
		if (offset&31) {
			uint8_t temp_buffer[33];
			int i = 1 + (offset & 31);
			status = i2cMasterTransmitTimeout(driver, GG_ADDR,
							&reg, sizeof(reg),
							temp_buffer, 33,
							tmo);
			while ((offset&31) && (size>0)) {
				*cdata++ = temp_buffer[i++];
				offset++;
				size--;
			}
		}
		else {
			uint8_t temp_buffer[33];
			int to_read;

			to_read = 33;
			if (size < 32)
				to_read = size+1;

			status = i2cMasterTransmitTimeout(driver, GG_ADDR,
							&reg, sizeof(reg),
							temp_buffer, to_read,
							tmo);
			_memcpy(cdata, temp_buffer+1, to_read-1);
			hack_if_necessary(driver, to_read);
			size  -= 32;
			cdata += 32;
		}
		if (status != RDY_OK)
			goto err;
		reg++;
	}

	i2cReleaseBus(driver);
	return 0;

err:
	i2cReleaseBus(driver);
	return status;
}

int gg_setdataflash(struct I2CDriver *driver,
		uint8_t subclass, uint8_t offset, void *data, int size) {
	msg_t status;
	uint8_t bfr[3];
	int ret;
	int ptr;
	int start = (offset/32);
	int end = (32*((offset+size)/32) + 32*(!!(offset+size)))/32;
	uint8_t eeprom_cache[256];

	if ((offset + size > subclass_size[subclass]) || size <= 0)
		return -1;

	chThdSleepMilliseconds(50);
	ret = gg_getdataflash(driver, subclass, 0,
			eeprom_cache, subclass_size[subclass]);
	if (ret < 0) {
		chprintf(STREAM, "Unable to read subclass %d: %d\r\n", subclass, i2cGetErrors(driver));
		return ret;
	}

	_memcpy(eeprom_cache + offset, data, size);

	i2cAcquireBus(driver);

	bfr[0] = 0x77; /* SetSubclassID register */
	bfr[1] = subclass;
	bfr[2] = subclass>>8;

	status = i2cMasterTransmitTimeout(driver, GG_ADDR,
					bfr, sizeof(bfr),
					NULL, 0,
					tmo);
	if (status < 0) {
		chprintf(STREAM, "Unable to set subclass %d: %d\r\n", subclass, i2cGetErrors(driver));
		goto err;
	}

	for (ptr = start; ptr < end; ptr++) {
		uint8_t temp_buffer[34];
		int write_size;

		write_size = 32;
		if ( (ptr + 1) * 32 > subclass_size[subclass])
			write_size = (subclass_size[subclass] & 31);

		/* Add an extra byte for the 'register' command */
		write_size++;

		/* Add an extra byte for the 'byte count' packet */
		write_size++;

		temp_buffer[0] = 0x78 + ptr;
		temp_buffer[1] = write_size - 2;
		_memcpy(temp_buffer + 2,
			eeprom_cache + (32 * ptr),
			write_size - 2);
		status = i2cMasterTransmitTimeout(driver, GG_ADDR,
					temp_buffer, write_size,
					NULL, 0,
					tmo);
		if (status != RDY_OK) {
			chprintf(STREAM, "Unable to update page 0x%x (subclass %d, offset %d): %d\r\n", temp_buffer[0], subclass, offset, i2cGetErrors(driver));
			goto err;
		}

		/* Let the flash write out */
		chThdSleepMilliseconds(100);
	}

	i2cReleaseBus(driver);
	return 0;

err:
	i2cReleaseBus(driver);
	return status;
}

int gg_setmanuf(struct I2CDriver *driver, uint8_t name[11]) {
	return gg_setdataflash(driver, 48, 26 + 1, name, 11);
}

int gg_setchem(struct I2CDriver *driver, uint8_t chem[4]) {
        return gg_setdataflash(driver, 48, 46 + 1, chem, 4);
}

int gg_setpuvthresh(struct I2CDriver *driver, uint16_t puv) {
	if (puv > 16000)
		return -1;
	return gg_setdataflash(driver, 0, 17, &puv, sizeof(puv));
}

int gg_setcells(struct I2CDriver *driver, int cells) {
	int ret;
	uint8_t cfg_a[2];

	if (cells < 2 || cells > 4)
		return -1;

	/* Set the number of cells */

	ret = gg_getdataflash(driver, 64, 0, cfg_a, sizeof(cfg_a));
	if (ret < 0)
		return ret;

	cfg_a[0] &= ~3;

	if (cells == 2)
		cfg_a[0] |= 1;
	else if (cells == 3)
		cfg_a[0] |= 2;
	else if (cells == 4)
		cfg_a[0] |= 3;
	else
		return -1;

	ret = gg_setdataflash(driver, 64, 0, cfg_a, sizeof(cfg_a));
	if (ret < 0)
		return ret;

	/* Set various over/undervoltage flags */

	ret = gg_setdataflash(driver, 0, 7, &cell_cfgs[cells].pov_threshold, 2);
	if (ret < 0)
		return ret;

	ret = gg_setdataflash(driver, 0, 10, &cell_cfgs[cells].pov_recovery, 2);
	if (ret < 0)
		return ret;

	ret = gg_setdataflash(driver, 0, 17, &cell_cfgs[cells].puv_threshold, 2);
	if (ret < 0)
		return ret;

	ret = gg_setdataflash(driver, 0, 20, &cell_cfgs[cells].puv_recovery, 2);
	if (ret < 0)
		return ret;

	ret = gg_setdataflash(driver, 16, 0, &cell_cfgs[cells].sov_threshold, 2);
	if (ret < 0)
		return ret;
	
	ret = gg_setdataflash(driver, 34, 2, &cell_cfgs[cells].charging_voltage, 2);
	if (ret < 0)
		return ret;
	
	ret = gg_setdataflash(driver, 38, 8, &cell_cfgs[cells].depleted_voltage, 2);
	if (ret < 0)
		return ret;
	
	ret = gg_setdataflash(driver, 38, 11, &cell_cfgs[cells].depleted_recovery, 2);
	if (ret < 0)
		return ret;
	
	ret = gg_setdataflash(driver, 48, 8, &cell_cfgs[cells].design_voltage, 2);
	if (ret < 0)
		return ret;
	
	ret = gg_setdataflash(driver, 68, 0, &cell_cfgs[cells].flash_update_ok_voltage, 2);
	if (ret < 0)
		return ret;
	
	ret = gg_setdataflash(driver, 68, 2, &cell_cfgs[cells].shutdown_voltage, 2);
	if (ret < 0)
		return ret;
	
	ret = gg_setdataflash(driver, 80, 45, &cell_cfgs[cells].term_voltage, 2);
	if (ret < 0)
		return ret;

	return 0;
}


static int gg_getword(struct I2CDriver *driver, uint8_t reg, void *word) {
	return gg_getblock(driver, reg, word, 2);
}

static int gg_getbyte(struct I2CDriver *driver, uint8_t reg, void *byte) {
	return gg_getblock(driver, reg, byte, 1);
}

static int gg_setblock(struct I2CDriver *driver, uint8_t reg,
			void *data, int size) {
	uint8_t bfr[size+1];
	msg_t status;

	bfr[0] = reg;
	_memcpy(bfr+1, data, size);

	i2cAcquireBus(driver);
	status = i2cMasterTransmitTimeout(driver, GG_ADDR,
					bfr, size+1,
					NULL, 0,
					tmo);
	i2cReleaseBus(driver);

	if (status != RDY_OK)
		return status;
	return 0;
}

static int gg_setword(struct I2CDriver *driver, uint8_t reg, uint16_t word) {
	return gg_setblock(driver, reg, &word, 2);
}

static int gg_setbyte(struct I2CDriver *driver, uint8_t reg, uint8_t byte) {
	return gg_setblock(driver, reg, &byte, 1);
}

static int gg_getstring(struct I2CDriver *driver, uint8_t addr,
			uint8_t *data, int size) {
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

int gg_percent(struct I2CDriver *driver, uint8_t *capacity) {
	return gg_getbyte(driver, sbs_data[REG_CAPACITY].addr, capacity);
}

int gg_cellvoltage(struct I2CDriver *driver, int cell, void *voltage) {
	if (cell <= 0 || cell > 4)
		return -1;
	cell--;
	cell = 3-cell;
	return gg_getword(driver, 0x3c+cell, voltage);
}

int gg_getmode(struct I2CDriver *driver, void *word) {
	return gg_getword(driver, BATTERY_MODE_OFFSET, word);
}

int gg_setprimary(struct I2CDriver *driver) {
	uint16_t word;
	int ret;
	ret = gg_getmode(driver, &word);
	if (ret < 0)
		return ret;
	word |= (1<<9);
	return gg_setword(driver, BATTERY_MODE_OFFSET, word);
}

int gg_setsecondary(struct I2CDriver *driver) {
	uint16_t word;
	int ret;
	ret = gg_getmode(driver, &word);
	if (ret < 0)
		return ret;
	word &= ~(1<<9);
	return gg_setword(driver, BATTERY_MODE_OFFSET, word);
}

int gg_temperature(struct I2CDriver *driver, int16_t *word) {
	int16_t *temp = word;
	int ret;
	ret = gg_getword(driver, sbs_data[REG_TEMPERATURE].addr, temp);
	if (ret < 0)
		return ret;
	*temp = *temp - 2730;
	return 0;
}

int gg_voltage(struct I2CDriver *driver, void *word) {
	return gg_getword(driver, sbs_data[REG_VOLTAGE].addr, word);
}

int gg_current(struct I2CDriver *driver, void *word) {
	int ret;
	ret = gg_getword(driver, sbs_data[REG_CURRENT].addr, word);
	if (ret < 0)
		return ret;
	return 0;
}

int gg_fullcapacity(struct I2CDriver *driver, int16_t *word) {
	int ret;
	ret = gg_getword(driver, sbs_data[REG_FULL_CHARGE_CAPACITY].addr, word);
	if (ret < 0)
		return ret;
	return 0;
}

int gg_average_current(struct I2CDriver *driver, void *word) {
	int ret;
	ret = gg_getword(driver, 0xb, word);
	if (ret < 0)
		return ret;
	return 0;
}

int gg_getstatus(struct I2CDriver *driver, void *word) {
	return gg_getword(driver, sbs_data[REG_STATUS].addr, word);
}

int gg_getfirmwareversion(struct I2CDriver *driver, void *word) {
	return gg_getmfgr(driver, 0x0001, word, 2);
}

int gg_getstate(struct I2CDriver *driver, void *word) {
	return gg_getmfgr(driver, 0x0006, word, 2);
}

int gg_setleds(struct I2CDriver *driver, int state) {
	switch(state) {
	case 1:
		return gg_getmfgr(driver, 0x0032, NULL, 0);
	case -1:
		return gg_getmfgr(driver, 0x0033, NULL, 0);
	default:
		return gg_getmfgr(driver, 0x0034, NULL, 0);
	}
}

int gg_calibrate(struct I2CDriver *driver) {
	return gg_getmfgr(driver, 0x0040, NULL, 0);
}

int gg_setchargecontrol(struct I2CDriver *driver, int state) {
	uint8_t reg;
	int ret;
	ret = gg_getbyte(driver, 0x03, &reg);
	if (ret < 0)
		return ret;
	if (state == 0) /* Inverse logic */
		reg |= 1<<6;
	else
		reg &= ~(1<<6);
	return gg_setbyte(driver, 0x03, reg);
}

int gg_forcedsg(struct I2CDriver *driver, int state) {
	uint16_t val;
	int ret;
	ret = gg_getword(driver, 0x46, &val);
	if (ret)
		return ret;
	if (state)
		val |= (1<<1);
	else
		val &= ~(1<<1);
	return gg_setword(driver, 0x46, val);
}

