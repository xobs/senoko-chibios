#ifndef __NOVENA_PMB_H__
#define __NOVENA_PMB_H__

struct I2CDriver;

void pmb_power_off(void);
void pmb_power_on(void);
int pmb_smbus_init(I2CDriver *driver);
int pmb_smbus_deinit(I2CDriver *driver);

#endif /* __NOVENA_PMB_H__ */
