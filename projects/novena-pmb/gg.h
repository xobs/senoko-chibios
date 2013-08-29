#ifndef __NOVENA_PMB_GG_H__
#define __NOVENA_PMB_GG_H__

struct I2CDriver;
int gg_init(struct I2CDriver *driver);
int gg_refresh(struct I2CDriver *driver, int property);
int gg_partname(struct I2CDriver *driver, int run, int reg, uint8_t name[8]);

#endif
