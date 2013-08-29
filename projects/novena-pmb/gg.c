#include <ch.h>
#include <hal.h>
#include <i2c.h>

#include <string.h>

struct I2CDriver;

#define GG_ADDR 0xb

int gg_init(struct I2CDriver *driver) {
	(void)driver;
	/* Pull GG_SYSPRES low to bring it out of reset */
	//palWritePad(GPIOA, PA11, 0);

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

int gg_partname(struct I2CDriver *driver, int run, int reg_, uint8_t name[8]) {
	msg_t status    = RDY_OK;
	systime_t tmo   = TIME_INFINITE; /* SMBus times out after 25ms in hardware */
	uint8_t reg     = reg_;
	i2caddr_t addr  = run;

	i2cAcquireBus(driver);
	status = i2cMasterTransmitTimeout(driver, addr,
					&reg, sizeof(reg),
					name, 8,
					tmo);
	i2cReleaseBus(driver);

	if (status != RDY_OK) {
		if (status == RDY_TIMEOUT)
			return -1;
		return i2cGetErrors(driver);
	}
	return 0;
}
