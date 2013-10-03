#include <ch.h>
#include <hal.h>
#include <i2c.h>

#include <string.h>

#include "chg.h"
#include "pmb.h"
#include "bionic.h"

#define CHG_ADDR 0x9

#define STREAM_SERIAL   (&SD1)
#define STREAM          ((BaseSequentialStream *)STREAM_SERIAL)

/* SMBus times out after 25ms in hardware */
static const systime_t tmo   = TIME_INFINITE;

#define THREAD_SLEEP 120000
static uint16_t g_current;
static uint16_t g_voltage;
static uint16_t g_input;

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


static int chg_getblock(struct I2CDriver *driver, uint8_t reg,
		void *data, int size)
{
	msg_t status;
	i2cAcquireBus(driver);
	status = i2cMasterTransmitTimeout(driver, CHG_ADDR,
					&reg, sizeof(reg),
					data, size,
					tmo);
	hack_if_necessary(driver, size);
	i2cReleaseBus(driver);
	if (status != RDY_OK)
		return status;
	return 0;
}

static int chg_setblock(struct I2CDriver *driver, void *data, int size) {
	msg_t status;

	status = i2cMasterTransmitTimeout(driver, CHG_ADDR,
					data, size,
					NULL, 0,
					tmo);

	if (status != RDY_OK) {
		if (status == RDY_TIMEOUT)
			return -1;
		return i2cGetErrors(driver) | 0x80000000;
	}

	return 0;
}

int chg_set(struct I2CDriver *driver, uint16_t current,
		uint16_t voltage, uint16_t input) {
	int ret;
	uint8_t bfr[3];

	i2cAcquireBus(driver);

	g_input = input;
	bfr[0] = 0x3f;
	bfr[1] = g_input;
	bfr[2] = g_input>>8;
	ret = chg_setblock(driver, bfr, sizeof(bfr));
	if (ret)
		goto out;

	g_current = current;
	bfr[0] = 0x14;
	bfr[1] = g_current;
	bfr[2] = g_current>>8;
	ret = chg_setblock(driver, bfr, sizeof(bfr));
	if (ret)
		goto out;

	g_voltage = voltage;
	bfr[0] = 0x15;
	bfr[1] = g_voltage;
	bfr[2] = g_voltage>>8;
	ret = chg_setblock(driver, bfr, sizeof(bfr));
	if (ret)
		goto out;

out:
	i2cReleaseBus(driver);
	return ret;
}

int chg_get(struct I2CDriver *driver, uint16_t *current,
		uint16_t *voltage, uint16_t *input) {
	chg_getblock(driver, 0x3f, input, 2);
	chg_getblock(driver, 0x14, current, 2);
	chg_getblock(driver, 0x15, voltage, 2);
	return 0;
}

int chg_getmanuf(struct I2CDriver *driver, uint16_t *word) {
	return chg_getblock(driver, 0xfe, word, 2);
}

int chg_getdevice(struct I2CDriver *driver, uint16_t *word) {
	return chg_getblock(driver, 0xff, word, 2);
}

static WORKING_AREA(chg_wa, 256);
static msg_t chg_thread(void *arg) {
	struct I2CDriver *driver = arg;
	chThdSleepMilliseconds(200);
	while (1) {
		chg_set(driver, g_current, g_voltage, g_input);
		chThdSleepMilliseconds(THREAD_SLEEP);
	}
	return 0;
}

void chg_runthread(struct I2CDriver *driver) {
	chThdCreateStatic(chg_wa, sizeof(chg_wa), HIGHPRIO, chg_thread, driver);
}
