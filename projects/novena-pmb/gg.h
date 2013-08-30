#ifndef __NOVENA_PMB_GG_H__
#define __NOVENA_PMB_GG_H__

struct I2CDriver;
int gg_init(struct I2CDriver *driver);
int gg_refresh(struct I2CDriver *driver, int property);
int gg_manuf(struct I2CDriver *driver, uint8_t *manuf);
int gg_partname(struct I2CDriver *driver, uint8_t name[8]);
int gg_chem(struct I2CDriver *driver, uint8_t *chem);
int gg_serial(struct I2CDriver *driver, void *serial);
int gg_percent(struct I2CDriver *driver, void *capacity);
int gg_cellvoltage(struct I2CDriver *driver, int cell, void *voltage);

#endif
