#include <ctype.h>
#include <stdio.h>
#include "nrf24.h"
#include "stm32l0xx_hal.h"

// config settings stored in eeprom
#define EEADDR_INT_MIN 16
#define EEADDR_INT_SEC 17
#define EE_INT_MIN  (*(uint8_t *)(DATA_EEPROM_BASE + EEADDR_INT_MIN))
#define EE_INT_SEC  (*(uint8_t *)(DATA_EEPROM_BASE + EEADDR_INT_SEC))

uint8_t unhex(char h, char l)
{
  if(h > 57) h -= 7;
  if(l > 57) l -= 7;
  return ((h-48)<<4)+(l-48);
}

void ee_set_interval(uint8_t min, uint8_t sec)
{
  HAL_FLASHEx_DATAEEPROM_Unlock();
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, DATA_EEPROM_BASE + EEADDR_INT_MIN, min);
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, DATA_EEPROM_BASE + EEADDR_INT_SEC, sec);
  HAL_FLASHEx_DATAEEPROM_Lock();
}

