#ifndef __NOVENA_PMB_GG_H__
#define __NOVENA_PMB_GG_H__

struct I2CDriver;
int gg_init(struct I2CDriver *driver);
int gg_refresh(struct I2CDriver *driver, int property);
int gg_manuf(struct I2CDriver *driver, uint8_t *manuf);
int gg_partname(struct I2CDriver *driver, uint8_t name[8]);
int gg_chem(struct I2CDriver *driver, uint8_t *chem);
int gg_serial(struct I2CDriver *driver, void *serial);
int gg_percent(struct I2CDriver *driver, uint8_t *capacity);
int gg_cellvoltage(struct I2CDriver *driver, int cell, void *voltage);
int gg_getmode(struct I2CDriver *driver, void *word);
int gg_setprimary(struct I2CDriver *driver);
int gg_setsecondary(struct I2CDriver *driver);
int gg_temperature(struct I2CDriver *driver, int16_t *word);
int gg_voltage(struct I2CDriver *driver, void *word);
int gg_current(struct I2CDriver *driver, void *word);
int gg_charging_voltage(struct I2CDriver *driver, void *word);
int gg_charging_current(struct I2CDriver *driver, void *word);
int gg_fullcapacity(struct I2CDriver *driver, int16_t *word);
int gg_designcapacity(struct I2CDriver *driver, int16_t *word);
int gg_current(struct I2CDriver *driver, void *word);
int gg_average_current(struct I2CDriver *driver, void *word);
int gg_getstatus(struct I2CDriver *driver, void *word);
int gg_getfirmwareversion(struct I2CDriver *driver, void *word);
int gg_getstate(struct I2CDriver *driver, void *word);
int gg_setleds(struct I2CDriver *driver, int state);
int gg_setchargecontrol(struct I2CDriver *driver, int state);
int gg_forcedsg(struct I2CDriver *driver, int state);
int gg_setmanuf(struct I2CDriver *driver, uint8_t name[11]);
int gg_setchem(struct I2CDriver *driver, uint8_t chem[4]);
int gg_setcells(struct I2CDriver *driver, int cells);
int gg_getcells(struct I2CDriver *driver, uint8_t *cells);
int gg_calibrate(struct I2CDriver *driver,
		int16_t voltage, int16_t current,
		uint16_t temperature, int cells);
int gg_setcapacity(struct I2CDriver *driver, int cells, uint16_t capacity);
int gg_setitenable(struct I2CDriver *driver);
int gg_timetoempty(struct I2CDriver *driver, uint16_t *minutes);
int gg_timetofull(struct I2CDriver *driver, uint16_t *minutes);


#endif
