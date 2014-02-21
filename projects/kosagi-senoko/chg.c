#include <ch.h>
#include <hal.h>
#include <i2c.h>

#include <string.h>

#include "chg.h"
#include "gg.h"
#include "pmb.h"
#include "bionic.h"

#define CHG_ADDR 0x9

#define STREAM_SERIAL   (&SD1)
#define STREAM          ((BaseSequentialStream *)STREAM_SERIAL)

/* SMBus times out after 25ms in hardware */
static const systime_t tmo   = TIME_INFINITE;

#define THREAD_SLEEP_MS 1100
#define MAX_ERRORS 10

/* If we're under 95%, charge the battery */
#define CAPACITY_TO_CHARGE 95

/* If the current is less than this, enable the charger */
#define CURRENT_TO_CHARGE 1

/* Rate (in mA) at which the battery can charge */
#define CHARGE_CURRENT 3000

/* Target voltage (in mV) of the battery */
#define CHARGE_VOLTAGE 12600

/* Rate (in mA) at which the wall can supply current */
#define CHARGE_WALL_CURRENT 3750

/* This is the ABSOLUTE MAXIMUM voltage allowed for each cell */
#define MV_MAX 4200
#define MV_MIN 3000
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


	if (current > 8064)
		return -1;

	if (voltage > 19200)
		return -1;

	if (input > 11004)
		return -1;

	current &= 0x1f80;

	voltage &= 0x7ff0;

	input >>= 1;
	input &= 0x1f80;

	i2cAcquireBus(driver);

	g_input = input;
	bfr[0] = 0x3f;
	bfr[1] = g_input;
	bfr[2] = g_input >> 8;
	ret = chg_setblock(driver, bfr, sizeof(bfr));
	if (ret)
		goto out;

	g_current = current;
	bfr[0] = 0x14;
	bfr[1] = g_current;
	bfr[2] = g_current >> 8;
	ret = chg_setblock(driver, bfr, sizeof(bfr));
	if (ret)
		goto out;

	g_voltage = voltage;
	bfr[0] = 0x15;
	bfr[1] = g_voltage;
	bfr[2] = g_voltage >> 8;
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
	*input <<= 1;
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
	int error_count = 0;

	chThdSleepMilliseconds(200);

	while (1) {
		int16_t cell_mv;
		int cell;
		int ret;
		uint8_t cell_count;
		uint8_t capacity;
		int16_t current;

		chThdSleepMilliseconds(THREAD_SLEEP_MS);


		/*
		 * Determine the battery's capacity and charge current,
		 * so we can decide if we need to charge or not.
		 */
		ret = gg_percent(driver, &capacity);
		if (ret < 0) {
			error_count++;
			capacity = CAPACITY_TO_CHARGE;
		}

		ret = gg_average_current(driver, &current);
		if (ret < 0) {
			error_count++;
			current = CURRENT_TO_CHARGE;
		}

		/* Charge the battery if necessary */
		if (capacity < CAPACITY_TO_CHARGE
		 && current < CURRENT_TO_CHARGE
		 && (g_current == 0 || g_voltage == 0)) {
			chg_set(driver, CHARGE_CURRENT,
					CHARGE_VOLTAGE,
					CHARGE_WALL_CURRENT);
			continue;
		}

		if (error_count > MAX_ERRORS)
			chg_set(driver, 0, 0, g_input << 1);


		/* Feed the charger watchdog timer */
		chg_set(driver, g_current, g_voltage, g_input << 1);
		ret = gg_getcells(driver, &cell_count);
		if (ret) {
			error_count++;
			continue;
		}

		/*
		 * Examine each cell to determine if it's undervoltage or
		 * overvoltage.  If it's undervoltage, cut power to the
		 * mainboard.  For overvoltage, stop charging.
		 */
		for (cell = 1; cell <= cell_count; cell++) {
			ret = gg_cellvoltage(driver, cell, &cell_mv);
			if (ret) {
				error_count++;
				continue;
			}

			/*
			 * If any one cell goes below the minimum, shut down
			 * everything.  These cells are very prone to expanding,
			 * and we don't want to stress them.
			 */
			if (cell_mv > 0 && cell_mv < MV_MIN
					&& (!g_current || !g_voltage))
				pmb_power_off();

			/* If we exceed the max voltage, shut down charging */
			if (cell_mv > MV_MAX && (g_current || g_voltage))
				chg_set(driver, 0, 0, g_input << 1);
		}

		error_count = 0;
	}
	return 0;
}

void chg_runthread(struct I2CDriver *driver) {
	chThdCreateStatic(chg_wa, sizeof(chg_wa), HIGHPRIO, chg_thread, driver);
}
