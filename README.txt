This is the working code for the l052/062 + BME280 + nrf24l01+ weather telemetry boards.

It uses l0 STOP mode, draws 4.5 uA after the first sleep, but after the second one
it goes up to 6.2 uA. But who cares, I guess. 

nrf24l01+ chips work great with external antennas and 250kbps (the lowest) data rate.

todo: 

Config setup? Can I bring up USB only if it's plugged in, and otherwise
not have it use power? And/or use a jumper to decide whether to bring up USB? 

#define EE_BATT_DIV (*(double *)(DATA_EEPROM_BASE + 0)) // 8 bytes long
char *ee_nrf_addr = (char *)(DATA_EEPROM_BASE + 8); // 8 bytes long
#define EE_INT_MIN  (*(uint8_t *)(DATA_EEPROM_BASE + 16))
#define EE_INT_SEC  (*(uint8_t *)(DATA_EEPROM_BASE + 17))
#define EE_NRF_CHAN (*(uint8_t *)(DATA_EEPROM_BASE + 18))
#define EE_NRF_PIPE (*(uint8_t *)(DATA_EEPROM_BASE + 19))
#define EE_NRF_BW   (*(uint8_t *)(DATA_EEPROM_BASE + 20))
#define EE_NRF_RETRY_COUNT (*(uint8_t *)(DATA_EEPROM_BASE + 21))
#define EE_NRF_RETRY_DELAY (*(uint8_t *)(DATA_EEPROM_BASE + 22))

char *ee_unit_id =  (char *)(DATA_EEPROM_BASE + 32);

ee_batt_div		 0 00	(8 bytes)		00 00 00 00 00 00 f0 3f
ee_nrf_addr		 8 08	(8 bytes)		00 00 00 de ad be ef 01
ee_int_min		16 10	(1 byte)                00 (this is in hex / BCD)
ee_int_sec		17 11	(1 byte)		31 (this is in hex / BCD)
ee_nrf_chan		18 12	(1 byte)                53 (dec 83)
ee_nrf_pipe		19 13	(1 byte)                00
ee_nrf_bw		20 14	(1 byte)		00
ee_nrf_retry_count 	21 15	(1 byte)		06
ee_nrf_retry_delay 	21 15	(1 byte)		07
ee_unit_ed		32 20	(null-terminated)	ZW

(in l062test) (for unit ZW) Hello! batt_divider=636.2562, unitid=[ZW]

(float 636.2562)
eewhex 00 e2 e9 95 b2 0c e2 83 40 00 00 00 de ad be ef 01 00 31 53 00 00 06 07
eewhex 20 00 00 00
eewrite 20 ZW

Hello! batt_divider=636.2562, unitid=[ZW]
ee_int_min = 00
ee_int_sec = 31
ee_nrf_chan = 83
ee_nrf_pipe = 0
ee_nrf_bw = 0
ee_nrf_retry_count = 6
ee_nrf_retry_delay= 7
ee_nrf_addr = 00 00 00 de ad be ef 01 


