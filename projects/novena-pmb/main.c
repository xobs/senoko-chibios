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
#include "bionic.h"

#define STREAM_SERIAL	(&SD1)
#define STREAM		((BaseSequentialStream *)STREAM_SERIAL)
#define SHELL_WA_SIZE	THD_WA_SIZE(2048)
#define I2C_BUS		(&I2CD2)

#define DAC_ADDR 0xd

#define CHARGE_VOLTAGE(cells) (4.2*cells)

static void cmd_i2c(BaseSequentialStream *chp, int argc, char **argv);
static void cmd_mode(BaseSequentialStream *chp, int argc, char **argv);
static void cmd_dac(BaseSequentialStream *chp, int argc, char **argv);
static void cmd_stats(BaseSequentialStream *chp, int argc, char **argv);
static void cmd_leds(BaseSequentialStream *chp, int argc, char **argv);
static void cmd_gg(BaseSequentialStream *chp, int argc, char **argv);
static void cmd_chg(BaseSequentialStream *chp, int argc, char **argv);

static char *permafailures[] = {
	"fuse is blown",
	"cell imbalance",
	"safety voltage failure",
	"FET failure",
};

static char *mfgr_states[] = {
	"wake up",
	"normal discharge",
	"???",
	"pre-charge",
	"???",
	"charge",
	"???",
	"charge termination",
	"fault charge terminate",
	"permanent failure",
	"overcurrent",
	"overtemperature",
	"battery failure",
	"sleep",
	"reserved",
	"battery removed",
};


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
	{"i2c",		cmd_i2c},
	{"mode",	cmd_mode},
	{"dac",		cmd_dac},
	{"stats",	cmd_stats},
	{"leds",	cmd_leds},
	{"gg",		cmd_gg},
	{"chg",		cmd_chg},
	{NULL,		NULL} /* Sentinal */
};

static const ShellConfig shell_cfg = {
	STREAM,
	commands
};

static void cmd_i2c(BaseSequentialStream *chp, int argc, char **argv) {
	chprintf(chp, "i2c(%d, %p)\r\n", argc, argv);
}

static void cmd_mode(BaseSequentialStream *chp, int argc, char **argv) {
//	chprintf(chp, "argc: %d  argv[0]: %s  argv[0][0]: %c\n", argc, argv[0], argv[0]?argv[0][0]:'.');
	if (argc == 1 && argv[0] && argv[0][0] == 'p') {
		chprintf(chp, "Setting primary mode...");
		gg_setprimary(I2C_BUS);
		chprintf(chp, " Set\r\n");
	}
	else if (argc == 1 && argv[0] && argv[0][0] == 's') {
		chprintf(chp, "Setting secondary mode...");
		gg_setsecondary(I2C_BUS);
		chprintf(chp, " Set\r\n");
	}
	else {
		uint16_t mode;
		gg_getmode(I2C_BUS, &mode);
		chprintf(chp, "Current mode:\r\n");
		chprintf(chp, "\tInternal charge controller%s supported\r\n",
				mode&(1<<0)?"":" NOT");
		chprintf(chp, "\tPrimary battery%s supported\r\n",
				mode&(1<<1)?"":" NOT");
		if (mode&(1<<7))
			chprintf(chp, "\tConditioning cycle requested\r\n");
		chprintf(chp, "\tInternal charge control %sabled\r\n",
				mode&(1<<8)?"EN":"DIS");
		chprintf(chp, "\t%s battery\r\n",
				mode&(1<<9)?"Primary":"Secondary");
		chprintf(chp, "\tAlarm broadcasts %sabled\r\n",
				mode&(1<<13)?"DIS":"EN");
		chprintf(chp, "\tCharge broadcasts %sabled\r\n",
				mode&(1<<14)?"DIS":"EN");
		chprintf(chp, "\tCapacity measured in %s\r\n",
				mode&(1<<15)?"10 mW":"mA");
	}
	return;
}

static void cmd_dac(BaseSequentialStream *chp, int argc, char **argv) {
	void *driver = I2C_BUS;
	systime_t timeout = TIME_INFINITE;
	uint16_t result;
	int status;
	(void)argc;
	(void)argv;

	pmb_smbus_init(driver);
	i2cAcquireBus(driver);
	status = i2cMasterReceiveTimeout(driver, DAC_ADDR,
			(void *)&result, sizeof(result),
			timeout);
	i2cReleaseBus(driver);
	pmb_smbus_deinit(driver);

	if (status != RDY_OK) {
		if (status == RDY_TIMEOUT)
			chprintf(chp, "Unable to read from DAC: timeout\r\n");
		else
			chprintf(chp, "Unable to read from DAC: 0x%x\r\n",
					i2cGetErrors(driver));
		return;
	}
	
	chprintf(chp, "DAC: 0x%04x (%d)\r\n", result, result);
	return;
}

static void cmd_stats(BaseSequentialStream *chp, int argc, char **argv) {
	uint8_t str[16];
	int16_t word = 0;
	uint8_t byte = 0;
	void *driver = I2C_BUS;
	int cell;
	int ret;
	(void)argc;
	(void)argv;

	ret = gg_manuf(driver, str);
	if (ret < 0)
		chprintf(chp, "Manufacturer:       error 0x%x\r\n", ret);
	else
		chprintf(chp, "Manufacturer:       %s\r\n", str);

	ret = gg_partname(driver, str);
	if (ret < 0)
		chprintf(chp, "Part name:          error 0x%x\r\n", ret);
	else
		chprintf(chp, "Part name:          %s\r\n", str);

	ret = gg_getfirmwareversion(driver, &word);
	if (ret < 0)
		chprintf(chp, "Firmware ver:       error 0x%x\r\n", ret);
	else
		chprintf(chp, "Firmware ver:       0x%04x\r\n", word);

	ret = gg_getstate(driver, &word);
	if (ret < 0)
		chprintf(chp, "State:              error 0x%x\r\n", ret);
	else {
		int chgfet, dsgfet;
		switch (word&0xc000) {
		case 0x0000:
			chgfet = 1;
			dsgfet = 1;
			break;
		case 0x4000:
			chgfet = 0;
			dsgfet = 1;
			break;
		case 0x8000:
			chgfet = 0;
			dsgfet = 0;
			break;
		default:
		case 0xc000:
			chgfet = 1;
			dsgfet = 0;
			break;
		}
		chprintf(chp, "Charge FET:         %s\r\n", chgfet?"on":"off");
		chprintf(chp, "Discharge FET:      %s\r\n", dsgfet?"on":"off");
		chprintf(chp, "State:              %s\r\n", mfgr_states[word&0xf]);
		if ((word&0xf) == 0x9)
			chprintf(chp, "PermaFailure:       %s\r\n",
					permafailures[word>>4&3]);
	}

	ret = gg_chem(driver, str);
	if (ret < 0)
		chprintf(chp, "Chemistry:          error 0x%x\r\n", ret);
	else
		chprintf(chp, "Chemistry:          %s\r\n", str);

	ret = gg_serial(driver, &word);
	if (ret < 0)
		chprintf(chp, "Serial number:      error 0x%x\r\n", ret);
	else
		chprintf(chp, "Serial number:      0x%04x\r\n", word);

	ret = gg_percent(driver, &byte);
	if (ret < 0)
		chprintf(chp, "Capacity:           error 0x%x\r\n", ret);
	else
		chprintf(chp, "Capacity:           %d%%\r\n", byte);

	ret = gg_fullcapacity(driver, &word);
	if (ret < 0)
		chprintf(chp, "Full Capacity:      error 0x%x\r\n", ret);
	else
		chprintf(chp, "Full Capacity:      %d mAh\r\n", word);

	ret = gg_temperature(driver, &word);
	if (ret < 0)
		chprintf(chp, "Temperature:        error 0x%x\r\n", ret);
	else
		chprintf(chp, "Temperature:        %d.%d C\r\n", word/10, word-(10*(word/10)));

	ret = gg_voltage(driver, &word);
	if (ret < 0)
		chprintf(chp, "Voltage:            error 0x%x\r\n", ret);
	else
		chprintf(chp, "Voltage:            %d mV\r\n", word);

	ret = gg_current(driver, &word);
	if (ret < 0)
		chprintf(chp, "Current:            error 0x%x\r\n", ret);
	else
		chprintf(chp, "Current:            %d mA\r\n", word);

	ret = gg_average_current(driver, &word);
	if (ret < 0)
		chprintf(chp, "Avg current:        error 0x%x\r\n", ret);
	else
		chprintf(chp, "Avg current:        %d mA\r\n", word);

	for (cell=1; cell<=4; cell++) {
		ret = gg_cellvoltage(driver, cell, &word);
		if (ret < 0)
			chprintf(chp, "Cell %d voltage:     error 0x%x\r\n",
					cell, ret);
		else
			chprintf(chp, "Cell %d voltage:     %d mV\r\n",
					cell, word);
	}

	chprintf(chp, "Alarms:\r\n");
	word = 0;
	ret = gg_getstatus(driver, &word);
	if (word & 0x8000)
		chprintf(chp, "    OVERCHARGED ALARM\r\n");
	if (word & 0x4000)
		chprintf(chp, "    TERMINATE CHARGE ALARM\r\n");
	if (word & 0x1000)
		chprintf(chp, "    OVER TEMP ALARM\r\n");
	if (word & 0x0800)
		chprintf(chp, "    TERMINATE DISCHARGE ALARM\r\n");
	if (word & 0x0200)
		chprintf(chp, "    REMAINING CAPACITY ALARM\r\n");
	if (word & 0x0100)
		chprintf(chp, "    REMAINING TIME ALARM\r\n");

	chprintf(chp, "Charge state:\r\n");
	if (word & 0x0080)
		chprintf(chp, "    Battery initialized\r\n");
	if (word & 0x0040)
		chprintf(chp, "    Battery discharging\r\n");
	if (word & 0x0020)
		chprintf(chp, "    Battery fully charged\r\n");
	if (word & 0x0010)
		chprintf(chp, "    Battery fully discharged\r\n");

	if (word & 0x000f)
		chprintf(chp, "STATUS ERROR CODE: 0x%x\r\n", word&0xf);
	else
		chprintf(chp, "No errors detected\r\n");

	return;
}

static void cmd_leds(BaseSequentialStream *chp, int argc, char **argv) {
	int ret;
	int state;

	if (argc == 1 && argv[0][0] == '+')
		state = 1;
	else if (argc == 1 && argv[0][0] == '-')
		state = -1;
	else
		state = 0;

	ret = gg_setleds(I2C_BUS, state);
	if (ret)
		chprintf(chp, "Unable to set LEDs: 0x%x\r\n", ret);
	else
		chprintf(chp, "LEDs set to %d\r\n", state);

	return;
}

static void cmd_gg(BaseSequentialStream *chp, int argc, char **argv) {
	int state;
	int ret;

	if (argc == 1 && argv[0][0] == '+')
		state = 1;
	else if (argc == 1 && argv[0][0] == '-')
		state = 0;
	else if (argc == 1 && !_strcasecmp(argv[0], "cal")) {
		chprintf(chp, "Calibrating... ");
		gg_calibrate(I2C_BUS);
		chprintf(chp, "Requested\r\n");
		return;
	}
	else {
		chprintf(chp,
			"Usage: gg +/- (enables/disables charge control)\r\n");
		return;
	}

	ret = gg_setchargecontrol(I2C_BUS, state);
	if (ret < 0)
		chprintf(chp, "Unable to set charge control: 0x%x\r\n", ret);
	else
		chprintf(chp, "Charge control %sabled\r\n", state>0?"EN":"DIS");
}

static void cmd_chg(BaseSequentialStream *chp, int argc, char **argv) {
	uint32_t current, voltage, input;
	int ret;

	if (argc == 0) {
		uint16_t word = 0;
		chprintf(chp, "Usage: chg [current] [voltage] [[input]]\r\n");
		chprintf(chp, "\tCurrent is measured in mA, voltage in mV\r\n");

		ret = chg_getmanuf(I2C_BUS, &word);
		if (ret < 0)
			chprintf(chp, "\tError getting manufacturer: 0x%x\r\n", ret);
		chprintf(chp, "\tChager manufacturer ID: 0x%04x\r\n", word);

		chg_getdevice(I2C_BUS, &word);
		chprintf(chp, "\tChager device ID: 0x%04x\r\n", word);
		return;
	}

	if (argc == 1) {
		chprintf(chp, "Disabling charging\r\n");
		chg_set(I2C_BUS, 0, 0, 0);
		return;
	}

	input = 1024; /* mA */
	current = _strtoul(argv[0], NULL, 0);
	voltage = _strtoul(argv[1], NULL, 0);
	if (argc > 2)
		input = _strtoul(argv[2], NULL, 0);

	/* Figure/check current */
	if (current > 4096) {
		chprintf(chp, "Error: That's too much current\r\n");
		return;
	}
	if (current < 256) {
		chprintf(chp, "Error: 256 mA is the minimum charge current\r\n");
		return;
	}
	current &= 0x1f80;

	/* Figure/check voltage */
	if (voltage > 16384) {
		chprintf(chp, "Error: That's an awful lot of voltage\r\n");
		return;
	}

	if (voltage < 1024) {
		chprintf(chp, "Error: Too little voltage (1024 mV min)\r\n");
		return;
	}
	voltage &= 0x7ff0;

	/* Figure/check input current */
	if (input > 11004) {
		chprintf(chp,
		"Error: 11004 mA is the max supported input current\r\n");
		return;
	}
	if (input < 256) {
		chprintf(chp, "Error: Input current must be at least 256 mA\r\n");
		return;
	}
	input >>= 1; /* 0b00000001 is 2 mA */
	input &= 0x1f80;

	chprintf(chp, "Setting charger: %dmA @ %dmV (input: %dmA)... ",
			current<<1, voltage, input<<1);
	ret = chg_set(I2C_BUS, current, voltage, input);
	if (ret < 0)
		chprintf(chp, "Error: 0x%x\r\n", ret);
	else
		chprintf(chp, "Ok\r\n");

	return;
}

int getVer(void) {
	return 9;
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

	pmb_smbus_init(I2C_BUS);
	gg_init(I2C_BUS);
	chThdSleepMilliseconds(200);

	sdStart(STREAM_SERIAL, &ser_cfg);

	chprintf(STREAM, "~Resetting (%d)~\r\n", getVer());
	chThdSleepMilliseconds(10);

	while (1) {
		if (!shell_thr)
			shell_thr = shellCreateStatic(&shell_cfg, shell_wa,
						SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminated(shell_thr)) {
			/* Recovers memory of the previous shell.   */
			chThdRelease(shell_thr);

			/* Triggers spawning of a new shell.        */
			shell_thr = NULL;

			chprintf(STREAM, "\r\nShell exited...\r\n");
		}
		chThdSleepMilliseconds(50);
	}

	return 0;
}
