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

int acks = 0;

/* Devices on the bus:
 * 0x09 - Battery charger (bq24765)
 * 0x0d - Gas gauge (bq20z95dbt)
 * 0x48 - Voltage monitor (dac081c085)
 */
uint8_t bus_devices[] = {
	0x09,
	0x0b,
	0x0d,
	0x48,
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
	int reg;
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

	/*
	 * Starts the transmission, it will be handled entirely in background.
	 */
	chprintf((BaseSequentialStream *)&SD1, "~Resetting~\r\n");
	chThdSleepMilliseconds(10);

	/*
	 * Normal main() thread activity, in this demo it does nothing.
	 */
	run = 0;
	reg = 0;
	while (TRUE) {
		int len = 0;
		char output[32];
		uint8_t msg[8];

		run &= 0x3;
		if (!run) {
			reg++;
			reg &= 0xff;
		}

		acks = gg_partname(&I2CD2, bus_devices[run], reg, msg);

		if (acks == 0)
			chprintf((BaseSequentialStream *)&SD1, "Hello: 0x%02x %02x 0x%08x [%c%c%c%c%c%c%c%c]\r\n",
				bus_devices[run],
				reg,
				*((uint32_t *)msg),
				msg[0],
				msg[1],
				msg[2],
				msg[3],
				msg[4],
				msg[5],
				msg[6],
				msg[7]);
		run++;

		chThdSleepMilliseconds(50);
	}

	return 0;
}
