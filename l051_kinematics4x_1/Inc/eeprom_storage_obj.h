/*
 * eeprom_storage_obj.h
 *
 *  Created on: Nov 6, 2016
 *      Author: root
 */

#ifndef INC_EEPROM_STORAGE_OBJ_H_
#define INC_EEPROM_STORAGE_OBJ_H_

#include "stdint.h"

// eeprom map *************************************************
// 0x08080000 mark (0x12345678)
// 0x08080004 gyro x calibration offset
// 0x08080008 gyro y calibration offset
// 0x0808000c gyro z calibration offset
// 0x08080010 accel x calibration offset
// 0x08080014 accel y calibration offset
// 0x08080018 accel z calibration offset
// 0x0808001c magnet x calibration offset
// 0x08080020 magnet y calibration offset
// 0x08080024 magnet z calibration offset

#define EEPROM_MARK (uint32_t *)0x12345678
#define EEPROM_BASE_ADDRESS (uint32_t *)0x08080000
#define EEPROM_MARK_ADDRESS (uint32_t *)0x08080000
#define EEPROM_GYRO_X (uint32_t *)0x08080004
#define EEPROM_GYRO_Y (uint32_t *)0x08080008
#define EEPROM_GYRO_Z (uint32_t *)0x0808000c
#define EEPROM_ACCEL_X (uint32_t *)0x08080010
#define EEPROM_ACCEL_Y (uint32_t *)0x08080014
#define EEPROM_ACCEL_Z (uint32_t *)0x08080018
#define EEPROM_MAGNET_X (uint32_t *)0x0808001c
#define EEPROM_MAGNET_Y (uint32_t *)0x08080020
#define EEPROM_MAGNET_Z (uint32_t *)0x08080024



void eeprom_write_mark();
void eeprom_clear_mark();
uint32_t eeprom_read_mark();
void eeprom_write_int32_value(int32_t value, uint32_t *address);
int32_t eeprom_read_int32_value(uint32_t *address);


#endif /* INC_EEPROM_STORAGE_OBJ_H_ */
