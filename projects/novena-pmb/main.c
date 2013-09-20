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
#include <chprintf.h>
#include <shell.h>

#include "gg.h"
#include "pmb.h"

#define STREAM_SERIAL	(&SD1)
#define STREAM		((BaseSequentialStream *)STREAM_SERIAL)
#define SHELL_WA_SIZE	THD_WA_SIZE(2048)

static void cmd_i2c(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_mode(BaseSequentialStream *chp, int argc, char *argv[]);

/*
 * Devices on the bus:
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

static const ShellCommand commands[] = {
	{"i2c", cmd_i2c},
	{"mode", cmd_mode},
	{NULL, NULL} /* Sentinal */
};

static const ShellConfig shell_cfg = {
	STREAM,
	commands
};

static void cmd_i2c(BaseSequentialStream *chp, int argc, char **argv) {
	chprintf(chp, "~?\r\n");
	chprintf(chp, "i2c(%d, %p)\r\n", argc, argv);
}

static void cmd_mode(BaseSequentialStream *chp, int argc, char **argv) {
	chprintf(chp, "!%\r\n");
	chprintf(chp, "mode(%d, %p)\r\n", argc, argv);
}

int getVer(void) {
	return 6;
}

static WORKING_AREA(shell_wa, SHELL_WA_SIZE);

/*
 * Application entry point.
 */
int main(void) {
	Thread *shell_thr = NULL;

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device
	 *   drivers and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread
	 *   and the RTOS is active.
	 */
	halInit();
	chSysInit();
	shellInit();

	pmb_smbus_init(&I2CD2);
	gg_init(&I2CD2);
	chThdSleepMilliseconds(200);

	sdStart(STREAM_SERIAL, &ser_cfg);

	chprintf(STREAM, "~Resetting (%d)~\r\n", getVer());
	chThdSleepMilliseconds(10);

	/*
	run = 0;
	while (TRUE) {
		uint8_t bfr[21];
		int size;
		uint8_t capacity;
		uint16_t word;
		int i;

		size = gg_partname(&I2CD2, bfr);
		chprintf(STREAM, "Part name (%d): %s\r\n", size, bfr);

		size = gg_manuf(&I2CD2, bfr);
		chprintf(STREAM, "Manufacturer name (%d): %s\r\n", size, bfr);

		size = gg_chem(&I2CD2, bfr);
		chprintf(STREAM, "Device chemistry (%d): %s\r\n", size, bfr);

		size = gg_serial(&I2CD2, &word);
		chprintf(STREAM, "Device serial (%d): %04x\r\n", size, word);

		size = gg_percent(&I2CD2, &capacity);
		chprintf(STREAM, "Battery charge (%d): %d%%\r\n", size, capacity);

		for (i=1; i<=4; i++) {
			size = gg_cellvoltage(&I2CD2, i, &word);
			chprintf(STREAM, "Cell %d voltage (%d): %d mV\r\n", i, size, word);
		}

		chThdSleepMilliseconds(500);
		run++;
	}
	*/

	while (1) {
		if (!shell_thr)
			shell_thr = shellCreateStatic(&shell_cfg, shell_wa, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminated(shell_thr)) {
			chThdRelease(shell_thr);    /* Recovers memory of the previous shell.   */
			shell_thr = NULL;           /* Triggers spawning of a new shell.        */
			chprintf(STREAM, "\r\nShell exited...\r\n");
			chThdSleepMilliseconds(50);
		}
		chThdSleepMilliseconds(50);
	}


	return 0;
}
