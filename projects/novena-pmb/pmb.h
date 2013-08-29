#ifndef __NOVENA_PMB_H__
#define __NOVENA_PMB_H__

struct I2CDriver;

void pmb_power_off(void);
void pmb_power_on(void);
void pmb_smbus_init(I2CDriver *driver);

#endif /* __NOVENA_PMB_H__ */
