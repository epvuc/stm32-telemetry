#include <string.h>
#include "main.h"
#include "stm32l0xx_hal.h"
#include "bme280.h"

extern I2C_HandleTypeDef hi2c1;

int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t sensor_data_forced_mode(struct bme280_dev *dev);
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev);
int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev);
void user_delay_ms(uint32_t ms);

struct bme280_dev bme280;

void bme280_once(char *buf, uint8_t len)
{
  int8_t rslt;
  uint8_t settings_sel;
  struct bme280_data comp_data;

  bme280.dev_id = BME280_I2C_ADDR_PRIM;
  bme280.intf = BME280_I2C_INTF;
  bme280.read = (void *)user_i2c_read;
  bme280.write = (void *)user_i2c_write;
  bme280.delay_ms = (void *)user_delay_ms;
  rslt = bme280_init(&bme280);
  bme280.settings.osr_h = BME280_OVERSAMPLING_2X;
  bme280.settings.osr_p = BME280_OVERSAMPLING_4X;
  bme280.settings.osr_t = BME280_OVERSAMPLING_4X;
  bme280.settings.filter = BME280_FILTER_COEFF_2;
  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
  rslt = bme280_set_sensor_settings(settings_sel, &bme280);
  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280);
  HAL_Delay(40);
  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280);
  rslt = bme280_set_sensor_mode(BME280_SLEEP_MODE, &bme280);
  snprintf(buf, len, "%0.2f %0.4f %0.1f",
	   comp_data.temperature,
	   comp_data.pressure/3386.3886,
	   comp_data.humidity);
}

void user_delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  HAL_StatusTypeDef status = HAL_OK;  

  status = HAL_I2C_Mem_Write(&hi2c1, dev_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 0xffff);
  if (status == HAL_OK)
    return(0);
  else
    return(-1);
}

int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 0xffff);
  if (status == HAL_OK) 
    return(0);
  else
    return(-1);
} 

