/*
 * Fake.h
 *
 *  Created on: Oct 9, 2021
 *      Author: leocelente
 */

#ifndef INC_FAKE_H_
#define INC_FAKE_H_

#include <stdint.h>
#include <stdio.h>
#include <cmsis_os2.h>

int sensor1_init(void);
int sensor2_init(void);
int storage1_init(void);
int storage2_init(void);

int sensor1_get(uint8_t *out_data);
int sensor2_get(uint8_t *out_data);
int storage1_save(size_t const size, uint8_t const data[size]);
int storage2_save(size_t const size, uint8_t const data[size]);

int actuator_get(void);
int actuator_set(int);

#endif /* INC_FAKE_H_ */
