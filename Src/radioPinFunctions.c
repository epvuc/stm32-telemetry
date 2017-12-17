/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* Please define your platform spesific functions in this file ...
* -----------------------------------------------------------------------------
*/
#include <stdint.h>
#include "stm32l0xx_hal.h"


/* ------------------------------------------------------------------------- */
void nrf24_setupPins()
{
  // this is already done by MX_GPIO_Init and MX_SPI_Init
}
/* ------------------------------------------------------------------------- */
void nrf24_ce_digitalWrite(uint8_t state)
{
    if(state)
    {
      HAL_GPIO_WritePin(RADIO_ENA_GPIO_Port, RADIO_ENA_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(RADIO_ENA_GPIO_Port, RADIO_ENA_Pin, GPIO_PIN_RESET);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_csn_digitalWrite(uint8_t state)
{
    if(state)
    {
      HAL_GPIO_WritePin(RADIO_CSN_GPIO_Port, RADIO_CSN_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(RADIO_CSN_GPIO_Port, RADIO_CSN_Pin, GPIO_PIN_RESET);
    }
}
/* ------------------------------------------------------------------------- */
