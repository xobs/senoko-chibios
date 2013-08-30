/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <ch.h>
#include <hal.h>
#include <uart.h>
#include <i2c.h>
#include <stdio.h>

#include "gg.h"
#include "pmb.h"
#include "chprintf.h"

/* Devices on the bus:
 * 0x09 - Battery charger (bq24765)
 * 0x0d - Gas gauge (bq20z95dbt)
 * 0x48 - Voltage monitor (dac081c085)
 */
uint8_t bus_devices[] = {
	0x09,	/* Battery charger (BQ24765) */
	0x0b,	/* Fuel gauge (BQ20Z95) */
	0x0d,	/* Voltage monitor (DAC081C081) */
	0x48,	/* ??? (Not a valid address) */
};

static const SerialConfig ser_cfg = {
	115200,
	0,
	0,
	0,
};



#if 0
static VirtualTimer vt1;

static WORKING_AREA(PollBattThreadWA, 256);
static msg_t PollBattThread(void *arg) {
	chRegSetThreadName("PollBatt");
	(void)arg;
	while (TRUE) {
		/*chThdSleepMilliseconds(rand() & 31);*/
		chThdSleepMilliseconds(32);
		//request_acceleration_data();
	}
	return 0;
}


static void restart(void *p) {
	(void)p;

}
#endif


/*
 * Application entry point.
 */
int main(void) {
	int run;
	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device
	 *   drivers and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread
	 *   and the RTOS is active.
	 */
	halInit();
	chSysInit();

	pmb_smbus_init(&I2CD2);
	gg_init(&I2CD2);
	chThdSleepMilliseconds(200);

	/*
	 * Activates the serial driver 1 using the driver default configuration.
	 */
	sdStart(&SD1, &ser_cfg);

	chprintf((BaseSequentialStream *)&SD1, "~Resetting~\r\n");
	chThdSleepMilliseconds(10);

	run = 0;
	while (TRUE) {
		uint8_t bfr[21];
		int size;
		uint8_t capacity;
		uint16_t word;
		int i;

		size = gg_partname(&I2CD2, bfr);
		chprintf((BaseSequentialStream *)&SD1, "Part name (%d): %s\r\n", size, bfr);

		size = gg_manuf(&I2CD2, bfr);
		chprintf((BaseSequentialStream *)&SD1, "Manufacturer name (%d): %s\r\n", size, bfr);

		size = gg_chem(&I2CD2, bfr);
		chprintf((BaseSequentialStream *)&SD1, "Device chemistry (%d): %s\r\n", size, bfr);

		size = gg_serial(&I2CD2, &word);
		chprintf((BaseSequentialStream *)&SD1, "Device serial (%d): %04x\r\n", size, word);

		size = gg_percent(&I2CD2, &capacity);
		chprintf((BaseSequentialStream *)&SD1, "Battery charge (%d): %d%%\r\n", size, capacity);

		for (i=1; i<=4; i++) {
			size = gg_cellvoltage(&I2CD2, i, &word);
			chprintf((BaseSequentialStream *)&SD1, "Cell %d voltage (%d): %d mV\r\n", i, size, word);
		}

			
		chThdSleepMilliseconds(50);
		run++;
	}

	return 0;
}
