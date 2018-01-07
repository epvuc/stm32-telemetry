/* Locations and defines of memory-mapped eeprom addresses for config data       */
/* On STM32L0, "eeprom" is fake and is actually bits of flash that are mapped    */
/* into the main address space, so while you need to write them with a special   */
/* write operation, to read them you can just use variables whose addresses are  */
/* the mapped "eeprom" addresses, you don't need to explicitly fetch at all.     */
#include "stm32l0xx_hal.h"

#define EEADDR_BATT_DIV		0
#define EEADDR_NRF_ADDR		8
#define EEADDR_INT_MIN		16
#define EEADDR_INT_SEC		17
#define EEADDR_NRF_CHAN		18
#define EEADDR_NRF_PIPE		19
#define EEADDR_NRF_BW		20
#define EEADDR_NRF_RETRY_COUNT	21
#define EEADDR_NRF_RETRY_DELAY	22
#define EEADDR_UNIT_ID		32

#define EE_BATT_DIV (*(double *)(DATA_EEPROM_BASE + EEADDR_BATT_DIV)) // 8 bytes long
#define EE_INT_MIN  (*(uint8_t *)(DATA_EEPROM_BASE + EEADDR_INT_MIN))
#define EE_INT_SEC  (*(uint8_t *)(DATA_EEPROM_BASE + EEADDR_INT_SEC))
#define EE_NRF_CHAN (*(uint8_t *)(DATA_EEPROM_BASE + EEADDR_NRF_CHAN))
#define EE_NRF_PIPE (*(uint8_t *)(DATA_EEPROM_BASE + EEADDR_NRF_PIPE))
#define EE_NRF_BW   (*(uint8_t *)(DATA_EEPROM_BASE + EEADDR_NRF_BW))
#define EE_NRF_RETRY_COUNT (*(uint8_t *)(DATA_EEPROM_BASE + EEADDR_NRF_RETRY_COUNT))
#define EE_NRF_RETRY_DELAY (*(uint8_t *)(DATA_EEPROM_BASE + EEADDR_NRF_RETRY_DELAY))

// The strings should be defined like this in source:
// char *ee_nrf_addr = (char *)(DATA_EEPROM_BASE + EEADDR_NRF_ADDR); // 8 bytes long
// char *ee_unit_id =  (char *)(DATA_EEPROM_BASE + EEADDR_UNIT_ID);
