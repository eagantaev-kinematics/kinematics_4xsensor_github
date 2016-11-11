/*
 * eeprom_storage_obj.c
 *
 *  Created on: Nov 6, 2016
 *      Author: root
 */

#include "eeprom_storage_obj.h"
#include "stm32l0xx_hal.h"


void eeprom_write_mark()
{
	HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_Erase(EEPROM_MARK_ADDRESS);
	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_MARK_ADDRESS, EEPROM_MARK);
	HAL_FLASHEx_DATAEEPROM_Lock();
}
void eeprom_clear_mark()
{
	HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_Erase(EEPROM_MARK_ADDRESS);
	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_MARK_ADDRESS, 0x00000000);
	HAL_FLASHEx_DATAEEPROM_Lock();
}
uint32_t eeprom_read_mark()
{
	uint32_t return_data = *((uint32_t *)0x08080000);
	//return *((uint32_t *)EEPROM_MARK_ADDRESS);
	return return_data;
}



void eeprom_write_int32_value(int32_t value, uint32_t *address)
{
	HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_Erase(address);
	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address, value);
	HAL_FLASHEx_DATAEEPROM_Lock();
}
int32_t eeprom_read_int32_value(uint32_t *address)
{
	return (int32_t)(*address);
}
