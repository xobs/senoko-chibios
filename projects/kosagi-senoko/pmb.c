#include <ch.h>
#include <hal.h>
#include <i2c.h>

#include "chg.h"

#define THREAD_SLEEP 1000

static const I2CConfig i2cfg2 = {
	OPMODE_SMBUS_HOST,
	100000,
	STD_DUTY_CYCLE,
};

static int power_state;

enum pmb_power_state {
	pmb_state_power_off = 0,
	pmb_state_power_on = 1,
};

static void pmb_set_power(enum pmb_power_state _power_state) {
	palWritePad(GPIOB, PB15, _power_state);
	power_state = _power_state;
	return;
}

void pmb_power_off(void) {
	pmb_set_power(pmb_state_power_off);
	return;
}

void pmb_power_on(void) {
	pmb_set_power(pmb_state_power_on);
	return;
}

int pmb_smbus_init(I2CDriver *driver) {
	i2cStart(driver, &i2cfg2);
	power_state = palReadPad(GPIOB, PB15);
	return 0;
}

int pmb_smbus_deinit(I2CDriver *driver) {
	i2cStop(driver);
	return 0;
}

void pmb_toggle_power(void) {
	if (power_state == pmb_state_power_on)
		pmb_power_off();
	else
		pmb_power_on();
	return;
}

static WORKING_AREA(pmb_wa, 256);
static msg_t pmb_thread(void *arg) {
	struct I2CDriver *driver = arg;
	while (1) {
		chThdSleepMilliseconds(200);
		if (palReadPad(GPIOA, PA0)) {
			pmb_power_off();
			chg_set(driver, 0, 0, 0);
		}
		chThdSleepMilliseconds(THREAD_SLEEP);
	}
	return 0;
}

void pmb_runthread(struct I2CDriver *driver) {
	chThdCreateStatic(pmb_wa, sizeof(pmb_wa), HIGHPRIO, pmb_thread, driver);
}
