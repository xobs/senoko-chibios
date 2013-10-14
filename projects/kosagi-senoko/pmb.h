#ifndef __NOVENA_PMB_H__
#define __NOVENA_PMB_H__

struct I2CDriver;

void pmb_power_off(void);
void pmb_power_on(void);
int pmb_smbus_init(I2CDriver *driver);
int pmb_smbus_deinit(I2CDriver *driver);
void pmb_runthread(struct I2CDriver *driver);
void pmb_toggle_power(void);

#endif /* __NOVENA_PMB_H__ */
