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
static const systime_t tmo   = TIME_INFINITE;

static const struct cell_cfg {
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

static const uint8_t subclass_size[] = {
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

static const struct gg_string {
	uint8_t size;
	uint8_t reg;
} gg_strings[] = {
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
	uint8_t *data8 = data;
	uint8_t tmp;

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

	if (data && size == 2) {
		tmp = data8[0];
		data8[0] = data8[1];
		data8[1] = tmp;
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

int gg_getflash(struct I2CDriver *driver,
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
	if (status != RDY_OK) {
		status = -1;
		goto err;
	}

	reg = (offset / 32) + 0x78;
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
				to_read = size + 1;

			status = i2cMasterTransmitTimeout(driver, GG_ADDR,
							&reg, sizeof(reg),
							temp_buffer, to_read,
							tmo);
			_memcpy(cdata, temp_buffer+1, to_read-1);
			hack_if_necessary(driver, to_read);
			size  -= 32;
			cdata += 32;
		}
		if (status != RDY_OK) {
			status = -2;
			goto err;
		}
		reg++;
	}

	i2cReleaseBus(driver);
	return 0;

err:
	i2cReleaseBus(driver);
	return status;
}

int gg_setflash(struct I2CDriver *driver,
		uint8_t subclass, uint8_t offset, void *data, int size) {
	msg_t status;
	uint8_t bfr[3];
	int ret;
	int ptr;
	int start = (offset/32);
	int end = (32*((offset+size)/32) + 32*(!!(offset+size)))/32;
	uint8_t eeprom_cache[256];

	if ((offset + size > subclass_size[subclass]) || size <= 0) {
		//chprintf(STREAM, "Size (%d) + offset (%d) > subclass size (%d)\r\n",
		//		size, offset, subclass_size[subclass]);
		return -1;
	}

	chThdSleepMilliseconds(50);
	ret = gg_getflash(driver, subclass, 0,
			eeprom_cache, subclass_size[subclass]);
	if (ret < 0) {
		//chprintf(STREAM, "Unable to read subclass %d: %d\r\n",
		//		subclass, i2cGetErrors(driver));
		return -2;
	}

	//chprintf(STREAM, "Original flash:\r\n");
	//print_hex(STREAM, eeprom_cache, subclass_size[subclass]);
	_memcpy(eeprom_cache + offset, data, size);
	//chprintf(STREAM, "New flash:\r\n");
	//print_hex(STREAM, eeprom_cache, subclass_size[subclass]);

	chThdSleepMilliseconds(25);

	i2cAcquireBus(driver);

	bfr[0] = 0x77; /* SetSubclassID register */
	bfr[1] = subclass;
	bfr[2] = subclass>>8;
	status = i2cMasterTransmitTimeout(driver, GG_ADDR,
					bfr, sizeof(bfr),
					NULL, 0,
					tmo);
	if (status < 0) {
		//chprintf(STREAM, "Unable to set subclass %d: %d\r\n",
		//		subclass, i2cGetErrors(driver));
		status = -3;
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

		//chprintf(STREAM, "Writing flash data:\r\n");
		//print_hex(STREAM, temp_buffer, write_size);
		status = i2cMasterTransmitTimeout(driver, GG_ADDR,
					temp_buffer, write_size,
					NULL, 0,
					tmo);
		if (status != RDY_OK) {
			//chprintf(STREAM, "Unable to update page 0x%x (subclass %d, offset %d): %d\r\n", temp_buffer[0], subclass, offset, i2cGetErrors(driver));
			status = -4;
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

int gg_setflash_word(struct I2CDriver *driver,
		uint8_t subclass, uint8_t offset, uint16_t data) {
	uint16_t val;
	val = ((data >> 8) & 0xff) | ((data << 8) & 0xff00);
	return gg_setflash(driver, subclass, offset, &val, 2);
}

int gg_getflash_word(struct I2CDriver *driver,
		uint8_t subclass, uint8_t offset, uint16_t *data) {
	int ret;
	ret = gg_getflash(driver, subclass, offset, data, 2);
	if (ret < 0)
		return ret;
	*data = ((*data >> 8) & 0xff) | ((*data << 8) & 0xff00);
	return 0;
}


int gg_setmanuf(struct I2CDriver *driver, uint8_t name[11]) {
	return gg_setflash(driver, 48, 26 + 1, name, 11);
}

int gg_setchem(struct I2CDriver *driver, uint8_t chem[4]) {
        return gg_setflash(driver, 48, 46 + 1, chem, 4);
}

int gg_setcells(struct I2CDriver *driver, int cells) {
	int ret;
	uint8_t cfg_a[2];

	if (cells < 2 || cells > 4)
		return -1;

	/* Set the number of cells */

	ret = gg_getflash(driver, 64, 0, cfg_a, sizeof(cfg_a));
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

	ret = gg_setflash(driver, 64, 0, cfg_a, sizeof(cfg_a));
	if (ret < 0)
		return ret;

	/* Set various over/undervoltage flags */
	
	ret = gg_setflash_word(driver, 0, 7, cell_cfgs[cells].pov_threshold);
	if (ret < 0)
		return ret;

	ret = gg_setflash_word(driver, 0, 10, cell_cfgs[cells].pov_recovery);
	if (ret < 0)
		return ret;

	ret = gg_setflash_word(driver, 0, 17, cell_cfgs[cells].puv_threshold);
	if (ret < 0)
		return ret;

	ret = gg_setflash_word(driver, 0, 20, cell_cfgs[cells].puv_recovery);
	if (ret < 0)
		return ret;

	ret = gg_setflash_word(driver, 16, 0, cell_cfgs[cells].sov_threshold);
	if (ret < 0)
		return ret;
	
	ret = gg_setflash_word(driver, 34, 2, cell_cfgs[cells].charging_voltage);
	if (ret < 0)
		return ret;
	
	ret = gg_setflash_word(driver, 38, 8, cell_cfgs[cells].depleted_voltage);
	if (ret < 0)
		return ret;
	
	ret = gg_setflash_word(driver, 38, 11, cell_cfgs[cells].depleted_recovery);
	if (ret < 0)
		return ret;
	
	ret = gg_setflash_word(driver, 48, 8, cell_cfgs[cells].design_voltage);
	if (ret < 0)
		return ret;
	
	ret = gg_setflash_word(driver, 68, 0, cell_cfgs[cells].flash_update_ok_voltage);
	if (ret < 0)
		return ret;
	
	ret = gg_setflash_word(driver, 68, 2, cell_cfgs[cells].shutdown_voltage);
	if (ret < 0)
		return ret;
	
	ret = gg_setflash_word(driver, 80, 45, cell_cfgs[cells].term_voltage);
	if (ret < 0)
		return ret;

	return 0;
}


int gg_getword(struct I2CDriver *driver, uint8_t reg, void *word) {
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
	if (data && size)
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

/*
static int gg_setbyte(struct I2CDriver *driver, uint8_t reg, uint8_t byte) {
	return gg_setblock(driver, reg, &byte, 1);
}

static int gg_setnull(struct I2CDriver *driver, uint8_t reg) {
	return gg_setblock(driver, reg, NULL, 0);
}
*/

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
	return gg_getword(driver, 0x1c, serial);
}

int gg_percent(struct I2CDriver *driver, uint8_t *capacity) {
	return gg_getbyte(driver, 0x0f, capacity);
}

int gg_cellvoltage(struct I2CDriver *driver, int cell, void *voltage) {
	if (cell <= 0 || cell > 4)
		return -1;
	cell--;
	cell = 3-cell;
	return gg_getword(driver, 0x3c+cell, voltage);
}

int gg_setcapacity(struct I2CDriver *driver, int cells, uint16_t capacity) {
	int cell;
	int ret;

	if (cells < 2 || cells > 4)
		return 1;

	/* Set capacity of known cells */
	for (cell=0; cell < cells; cell++) {
		if (cell < cells)
			ret = gg_setflash_word(driver, 82, cell*2, capacity);
		else
			/* Set other capacities to 0 */
			ret = gg_setflash_word(driver, 82, cell*2, 0);
		if (ret < 0)
			return ret;
	}

	/* Set Qmax Pack */
	ret = gg_setflash_word(driver, 82, 8, capacity);
	if (ret < 0)
		return ret;

	/* Set the SBS value */
	ret = gg_setflash_word(driver, 48, 22, capacity);
	if (ret < 0)
		return ret;

	/* Set tracking support */
	uint8_t reg;
	reg = 0x03;
	ret = gg_setflash(driver, 82, 12, &reg, 1);
	if (ret < 0)
		return ret;

	return 0;
}


int gg_getmode(struct I2CDriver *driver, void *word) {
	return gg_getword(driver, 0x03, word);
}

int gg_setprimary(struct I2CDriver *driver) {
	uint8_t reg[2];
	int ret;
	ret = gg_getmode(driver, reg);
	if (ret < 0)
		return ret;
	reg[0] |= (1<<1);
	return gg_setblock(driver, 0x03, reg, 2);
}

int gg_setsecondary(struct I2CDriver *driver) {
	uint8_t reg[2];
	int ret;
	ret = gg_getmode(driver, reg);
	if (ret < 0)
		return ret;
	reg[0] &= ~(1<<1);
	return gg_setblock(driver, 0x03, reg, 2);
}

int gg_temperature(struct I2CDriver *driver, int16_t *word) {
	int16_t *temp = word;
	int ret;
	ret = gg_getword(driver, 0x08, temp);
	if (ret < 0)
		return ret;
	*temp = *temp - 2730;
	return 0;
}

int gg_timetofull(struct I2CDriver *driver, uint16_t *minutes) {
	return gg_getword(driver, 0x13, minutes);
}

int gg_voltage(struct I2CDriver *driver, void *word) {
	return gg_getword(driver, 0x09, word);
}

int gg_current(struct I2CDriver *driver, void *word) {
	return gg_getword(driver, 0x0a, word);
}

int gg_charging_current(struct I2CDriver *driver, void *word) {
	return gg_getword(driver, 0x14, word);
}

int gg_charging_voltage(struct I2CDriver *driver, void *word) {
	return gg_getword(driver, 0x15, word);
}

int gg_fullcapacity(struct I2CDriver *driver, int16_t *word) {
	return gg_getword(driver, 0x10, word);
}

int gg_designcapacity(struct I2CDriver *driver, int16_t *word) {
	return gg_getword(driver, 0x18, word);
}

int gg_average_current(struct I2CDriver *driver, void *word) {
	int ret;
	ret = gg_getword(driver, 0xb, word);
	if (ret < 0)
		return ret;
	return 0;
}

int gg_getstatus(struct I2CDriver *driver, void *word) {
	return gg_getword(driver, 0x16, word);
}

int gg_getfirmwareversion(struct I2CDriver *driver, void *word) {
	return gg_getmfgr(driver, 0x0001, word, 2);
}

int gg_getstate(struct I2CDriver *driver, void *word) {
	int ret;
	ret = gg_getmfgr(driver, 0x0006, word, 2);
	if (ret < 0)
		return ret;
	return 0;
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

int gg_setitenable(struct I2CDriver *driver) {
	return gg_getmfgr(driver, 0x0021, NULL, 0);
}

#if 0
int gg_calibrate(struct I2CDriver *driver,
		int16_t voltage, int16_t current,
		uint16_t temperature, int cells) {
	int ret;
	ret = gg_getmfgr(driver, 0x0040, NULL, 0);
	if (ret < 0)
		return ret;

	ret = gg_setword(driver, 0x63, cells);
	if (ret < 0) {
		chprintf(STREAM, "Unable to set number of cells\r\n");
		goto out;
	}

	ret = gg_setword(driver, 0x60, current);
	if (ret < 0) {
		chprintf(STREAM, "Unable to set current\r\n");
		goto out;
	}

	ret = gg_setword(driver, 0x61, voltage);
	if (ret < 0) {
		chprintf(STREAM, "Unable to set voltage\r\n");
		goto out;
	}

	ret = gg_setword(driver, 0x62, temperature);
	if (ret < 0) {
		chprintf(STREAM, "Unable to set temperature\r\n");
		goto out;
	}

	/* Start calibration */
	ret = gg_setword(driver, 0x51, 0xc0d5);
	int tries;
	for (tries=0; tries<10; tries++) {
		uint16_t val;
		chThdSleepMilliseconds(200);
		ret = gg_getword(driver, 0x52, &val);
		if (ret < 0) {
			chprintf(STREAM, "Unable to query: %d\r\n", ret);
			continue;
		}
		chprintf(STREAM, "Val: 0x%x\r\n", val);
		if (val & 0x3fff)
			break;
	}

	/* Write results to flash */


	return gg_setnull(driver, 0x73);

out:
	gg_setnull(driver, 0x73);
	return ret;
}
#endif

int gg_setchargecontrol(struct I2CDriver *driver, int state) {
	uint8_t reg[2];
	int ret;

#if 0
	/* Disable the feature in flash, if necessary */
	ret = gg_getflash(driver, 64, 2, reg, 2);
	if (ret < 0)
		return ret;

	if (reg[1] & 1) {
		reg[1] &= ~1;
		ret = gg_setflash(driver, 64, 2, reg, 2);
	}
#endif

	/* Turn on charge control */
	ret = gg_getblock(driver, 0x03, reg, 2);
	if (ret < 0)
		return ret;
	if (state == 0) /* Inverse logic */
		reg[0] |= 1<<6;
	else
		reg[0] &= ~(1<<6);
	ret = gg_setblock(driver, 0x03, reg, 2);
	if (ret < 0)
		return ret;

	return ret;
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
