
#include <ch.h>
#include <hal.h>
#include <i2c.h>

static const I2CConfig i2cfg2 = {
	OPMODE_SMBUS_HOST,
	100000,
	STD_DUTY_CYCLE,
};


enum pmb_power_state {
	pmb_state_power_off = 0,
	pmb_state_power_on = 1,
};




static void pmb_set_power(enum pmb_power_state power_state) {
	palWritePad(GPIOB, PB15, power_state);
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
	return 0;
}

int pmb_smbus_deinit(I2CDriver *driver) {
	i2cStop(driver);
	return 0;
}
