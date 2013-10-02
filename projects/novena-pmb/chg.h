#ifndef __NOVENA_PMB_CHG_H__
#define __NOVENA_PMB_CHG_H__

struct I2CDriver;
int chg_set(struct I2CDriver *driver, uint32_t current,
		uint32_t voltage, uint32_t input);
int chg_get(struct I2CDriver *driver, uint16_t *current,
		uint16_t *voltage, uint16_t *input);
int chg_getmanuf(struct I2CDriver *driver, uint16_t *word);
int chg_getdevice(struct I2CDriver *driver, uint16_t *word);
#endif
