/*
 * Fake.c
 *
 *  Created on: Oct 9, 2021
 *      Author: leocelente
 */
#include "Fake.h"
#include "zenith.h"

int sensor1_init(void) {
	osDelay(2);
	HAL_Delay(2);
	return 0;
}

int sensor1_get(uint8_t *out_data) {
	osDelay(5);
	HAL_Delay(2);
	debug("Got System Data");
	return 0;
}
// -----------------------------------
int sensor2_init(void) {
	osDelay(1);
	return 0;
}

int sensor2_get(uint8_t *out_data) {
	osDelay(2);
	HAL_Delay(2);
	debug("Got IMU Data");
	return 0;
}
// -----------------------------------
// -----------------------------------
int storage1_init(void) {
	osDelay(1);
	return 0;
}

int storage1_save(size_t const size, uint8_t const data[size]) {
	osDelay(10);
	HAL_Delay(2);
	debug("Saved on FLASH");
	return 0;
}
// -----------------------------------
int storage2_init(void) {
	osDelay(10);
	return 0;
}

int storage2_save(size_t const size, uint8_t const data[size]) {
	osDelay(10);
	HAL_Delay(2);
	debug("Saved on SDCard");
	return 0;
}
// -----------------------------------
// -----------------------------------
int actuator_get() {
//	debug("Actuator Get");
	return 1;
}

int actuator_set(int i) {
//	debug("Actuator set");
	return 0;
}
