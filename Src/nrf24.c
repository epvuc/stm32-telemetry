/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* This library is based on this library: 
*   https://github.com/aaronds/arduino-nrf24l01
* Which is based on this library: 
*   http://www.tinkerer.eu/AVRLib/nRF24L01
* -----------------------------------------------------------------------------
*/
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "nrf24.h"
#include "stm32l0xx_hal.h"
// config settings stored in eeprom
#include "config_eeprom.h"
char *ee_nrf_addr = (char *)(DATA_EEPROM_BASE + 8); // 8 bytes long
extern char *ee_unit_id;

extern SPI_HandleTypeDef hspi1;

extern uint8_t spi_was_changed;

void nrf24_set_ack_payload(uint8_t pipe, uint8_t *buf, uint8_t len);
void nrf24_featureActivate();
void led_on(void);
void led_off(void);
// from misc.c:
void ee_set_interval(uint8_t min, uint8_t sec);
uint8_t unhex(char h, char l);

uint8_t spi_transfer(uint8_t dat)
{
  uint8_t rxbuf = 0;
  HAL_SPI_TransmitReceive(&hspi1, &dat, &rxbuf, 1, 5000);
  return (uint8_t)rxbuf;
}

/* init the hardware pins */
void nrf24_init() 
{
  nrf24_setupPins();
  // spi clock needs to be slow for the nrf chip
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  HAL_SPI_Init(&hspi1);
  nrf24_ce_digitalWrite(LOW);
  nrf24_csn_digitalWrite(HIGH);    
}

void nrf24_deinit() 
{
  // this sets a global from main.c which the cli loop in cli.c checks to see if it
  // might need to reset the spi bus config and remount the chanfat volumes 
  spi_was_changed = 1;
}

void my_config(void)
{
  uint8_t bwconf;
  
  nrf24_configRegister(RF_CH, EE_NRF_CHAN);

  if (EE_NRF_BW == 0) 
    bwconf = (0<<RF_DR_HIGH)|(1<<RF_DR_LOW);
  else if (EE_NRF_BW == 1)
    bwconf = (0<<RF_DR_HIGH)|(0<<RF_DR_LOW);
  else if (EE_NRF_BW == 2)
    bwconf = (1<<RF_DR_HIGH)|(0<<RF_DR_LOW);
  else
    bwconf = (0<<RF_DR_HIGH)|(1<<RF_DR_LOW); // default to 250khz bw i guess

  nrf24_configRegister(RF_SETUP, bwconf|((0x03)<<RF_PWR));
  nrf24_configRegister(CONFIG, nrf24_CONFIG);
  nrf24_configRegister(SETUP_RETR, ((EE_NRF_RETRY_DELAY & 0x0F) << ARD) | ((EE_NRF_RETRY_COUNT & 0x0F) << ARC));
  nrf24_configRegister(EN_AA,(1<<ENAA_P0)|(0<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));
  nrf24_configRegister(EN_RXADDR,(1<<ERX_P0)|(0<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));
  nrf24_configRegister(DYNPD,(1<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));
  nrf24_configRegister(FEATURE,(1<<EN_DPL)|(1<<EN_ACK_PAY));
  nrf24_featureActivate();
  nrf24_powerUpRx();
}

/* Set the RX address */
void nrf24_rx_address(uint8_t * adr, uint8_t pipe) 
{
    nrf24_ce_digitalWrite(LOW);
    nrf24_writeRegister(pipe ,adr,nrf24_ADDR_LEN);
    nrf24_ce_digitalWrite(HIGH);
}

/* Set the TX address */
void nrf24_tx_address(uint8_t* adr)
{
    nrf24_writeRegister(TX_ADDR,adr,nrf24_ADDR_LEN);
}

/* Checks if data is available for reading */
/* Returns 1 if data is ready ... */
uint8_t nrf24_dataReady() 
{
    // See note in getData() function - just checking RX_DR isn't good enough
    uint8_t status = nrf24_getStatus();

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RX_DR) ) 
    {
        return 1;
    }

    return !nrf24_rxFifoEmpty();;
}

/* Checks if receive FIFO is empty or not */
uint8_t nrf24_rxFifoEmpty()
{
    uint8_t fifoStatus;

    nrf24_readRegister(FIFO_STATUS,&fifoStatus,1);
    
    return (fifoStatus & (1 << RX_EMPTY));
}

/* Returns the length of data waiting in the RX fifo */
uint8_t nrf24_payloadLength()
{
    uint8_t status;
    nrf24_csn_digitalWrite(LOW);
    spi_transfer(R_RX_PL_WID);
    status = spi_transfer(0x00);
    nrf24_csn_digitalWrite(HIGH);
    return status;
}

/* Reads payload bytes into data array */
void nrf24_getData(uint8_t* data, uint8_t len) 
{
  uint8_t i;
    /* Pull down chip select */
    nrf24_csn_digitalWrite(LOW);                               

    /* Send cmd to read rx payload */
    spi_transfer( R_RX_PAYLOAD );
    
    /* Read payload */
    for (i=0; (i<len); i++) 
	data[i] = spi_transfer(0x00);

    /* Pull up chip select */
    nrf24_csn_digitalWrite(HIGH);

    /* Reset status register */
    nrf24_configRegister(STATUS,(1<<RX_DR));   
}

/* Returns the number of retransmissions occured for the last message */
uint8_t nrf24_retransmissionCount()
{
    uint8_t rv;
    nrf24_readRegister(OBSERVE_TX,&rv,1);
    rv = rv & 0x0F;
    return rv;
}

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void nrf24_send(uint8_t* value, uint8_t len) 
{    
    /* Go to Standby-I first */
    nrf24_ce_digitalWrite(LOW);
     
    /* Set to transmitter mode , Power up if needed */
    nrf24_powerUpTx();

    /* Do we really need to flush TX fifo each time ? */
    #if 1
        /* Pull down chip select */
        nrf24_csn_digitalWrite(LOW);           

        /* Write cmd to flush transmit FIFO */
        spi_transfer(FLUSH_TX);     

        /* Pull up chip select */
        nrf24_csn_digitalWrite(HIGH);                    
    #endif 

    /* Pull down chip select */
    nrf24_csn_digitalWrite(LOW);

    /* Write cmd to write payload */
    spi_transfer(W_TX_PAYLOAD);

    /* Write payload */
    nrf24_transmitSync(value, len);   

    /* Pull up chip select */
    nrf24_csn_digitalWrite(HIGH);

    /* Start the transmission */
    nrf24_ce_digitalWrite(HIGH);    
}

uint8_t nrf24_isSending()
{
    uint8_t status;

    /* read the current status */
    status = nrf24_getStatus();
                
    /* if sending successful (TX_DS) or max retries exceded (MAX_RT). */
    if((status & ((1 << TX_DS)  | (1 << MAX_RT))))
    {        
        return 0; /* false */
    }

    return 1; /* true */

}

uint8_t nrf24_getStatus()
{
    uint8_t rv;
    nrf24_csn_digitalWrite(LOW);
    rv = spi_transfer(NOP);
    nrf24_csn_digitalWrite(HIGH);
    return rv;
}

uint8_t nrf24_lastMessageStatus()
{
    uint8_t rv;

    rv = nrf24_getStatus();

    /* Transmission went OK */
    if((rv & ((1 << TX_DS))))
    {
        return NRF24_TRANSMISSON_OK;
    }
    /* Maximum retransmission count is reached */
    /* Last message probably went missing ... */
    else if((rv & ((1 << MAX_RT))))
    {
        return NRF24_MESSAGE_LOST;
    }  
    /* Probably still sending ... */
    else
    {
        return 0xFF;
    }
}

void nrf24_powerUpRx()
{     
    nrf24_csn_digitalWrite(LOW);
    spi_transfer(FLUSH_RX);
    nrf24_csn_digitalWrite(HIGH);

    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 

    nrf24_ce_digitalWrite(LOW);    
    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(1<<PRIM_RX)));    
    nrf24_ce_digitalWrite(HIGH);
}

void nrf24_powerUpTx()
{
    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 

    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(0<<PRIM_RX)));
}

void nrf24_powerDown()
{
    nrf24_ce_digitalWrite(LOW);
    nrf24_configRegister(CONFIG,nrf24_CONFIG);
}


/* send and receive multiple bytes over SPI */
void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len)
{
  uint8_t i;

  for(i=0;i<len;i++) 
    datain[i]= spi_transfer(dataout[i]);
}

/* send multiple bytes over SPI */
void nrf24_transmitSync(uint8_t* dataout,uint8_t len)
{
    uint8_t i;
    
    for(i=0;i<len;i++)
    {
        spi_transfer(dataout[i]);
    }

}

/* Clocks only one byte into the given nrf24 register */
void nrf24_configRegister(uint8_t reg, uint8_t value)
{
    nrf24_csn_digitalWrite(LOW);
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(value);
    nrf24_csn_digitalWrite(HIGH);
}

/* Read single register from nrf24 */
void nrf24_readRegister(uint8_t reg, uint8_t* value, uint8_t len)
{
    nrf24_csn_digitalWrite(LOW);
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    nrf24_transferSync(value,value,len);
    nrf24_csn_digitalWrite(HIGH);
}

/* Write to a single register of nrf24 */
void nrf24_writeRegister(uint8_t reg, uint8_t* value, uint8_t len) 
{
    nrf24_csn_digitalWrite(LOW);
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    nrf24_transmitSync(value,len);
    nrf24_csn_digitalWrite(HIGH);
}

void nrf24_set_ack_payload(uint8_t pipe, uint8_t *buf, uint8_t len)
{
  uint8_t i;
  nrf24_csn_digitalWrite(LOW);
  spi_transfer(0xA8 | (pipe &0x07));
  for (i=0; (i<len) ; i++)
    spi_transfer(buf[i]);
  nrf24_csn_digitalWrite(HIGH);    
}

void nrf24_featureActivate()
{
  nrf24_csn_digitalWrite(LOW);
  spi_transfer(0x50);
  spi_transfer(0x73);
  nrf24_csn_digitalWrite(HIGH);
}

uint8_t tx_address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01};
uint8_t rx_address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01};



int send_radio_message(char *msg, uint8_t len)
{
  int t=0;
  uint8_t status, payload_length;
  uint8_t int_min, int_sec;
  uint8_t ackpl[32];
  if (len > 32) len = 32;
  nrf24_init();
  my_config();
  nrf24_tx_address(tx_address);
  nrf24_rx_address(rx_address, RX_ADDR_P0);

  nrf24_send((uint8_t *)msg, len);
  while(nrf24_isSending() && (t++ < 100)) HAL_Delay(1);
  if (t>=100)  {
    nrf24_powerDown();
    //    printf("nrf timeout\r\n");
    return(-1);
  }
  status = nrf24_lastMessageStatus();
  if (status == NRF24_TRANSMISSON_OK) {
    // yay, it worked. (blink once)
    led_on(); HAL_Delay(10); led_off();
  } else if (status == NRF24_MESSAGE_LOST) {
    // sad day, message lost (blink twice)
    // this one delay accounts for the majority of the total power use and months or years of run time, haha
    // just so you can see a double flash if it failed. 
    led_on(); HAL_Delay(10); led_off(); HAL_Delay(100);
    led_on(); HAL_Delay(10); led_off();
  } else {
    // this is no big deal because it never really happens
    led_on(); HAL_Delay(20); led_off(); HAL_Delay(150);
    led_on(); HAL_Delay(20); led_off(); HAL_Delay(150);
    led_on(); HAL_Delay(20); led_off();
  } 
  status = nrf24_retransmissionCount();  // this is how many tries it took to get through

  // From here, we've already sent our data packet. The rest is just to see if the receiving
  // end sent us an ACK payload, and if so, if we want to do something with it. 
  if (nrf24_dataReady()) {
    // the recipient had a return payload for us! ooooh!
    payload_length = nrf24_payloadLength();
    nrf24_getData(ackpl, payload_length);
    // the contents of the ACK payload are in ackpl, length
    // for now, just interpret a command to set the wakeup interval
    if(payload_length > 7) {
      if(strncmp((char *)ackpl, ee_unit_id, 2) == 0) {
	// expect command format IMMSS where MM and SS are BCD 
	// You can lock yourself out for a long time if you set this wrong.
	// nfi what happens if you set it to 0, 0. 
	if(ackpl[3] == 'I') { 
	  int_min = unhex(ackpl[4], ackpl[5]);
	  int_sec = unhex(ackpl[6], ackpl[7]);
	  if ((int_min+int_sec) > 0) { 
	    ee_set_interval(int_min, int_sec);
	    snprintf((char *)ackpl, 32, "%s_int_set_%02xm_%02xs", ee_unit_id, EE_INT_MIN, EE_INT_SEC);
	  } else {
	    snprintf((char *)ackpl, 32, "%s_int_set_bad_param", ee_unit_id);
	  }
	} else {
	  snprintf((char *)ackpl, 32, "%s_GOT_IT!", ee_unit_id);
	}
	// make sure you don't make this > 32 chars. snprintf will force a null terminator but it'll truncate
	nrf24_send(ackpl, strlen((char *)ackpl));
	t=0; while(nrf24_isSending() && (t++ < 100)) HAL_Delay(1);
      }
    }
  }
  nrf24_powerDown();
  return(status);
}


/* test funcs */
#ifdef NRF_TEST_FUNCTIONS
void nrf_tx(void)
{
  uint32_t t;
  uint16_t x = 0;
  uint8_t status, i, payload_length;
  char txbuf[32], ackpl[32];
  
  nrf24_init();
  my_config(); 
  nrf24_tx_address(tx_address);
  nrf24_rx_address(rx_address, RX_ADDR_P0);

  while(1) {
    if (VCP_read(&status, 1) == 1) {
      nrf24_deinit();
      return;
    }

    /* Prepare message and start send. Packet length (<=32) is automatically set */
    snprintf(txbuf, sizeof(txbuf)-1, "HeEpY:%05u", x++);
    //    printf("Sending: [%s]... ", txbuf);
    nrf24_send(txbuf, strlen(txbuf));
    
    t = 0;
    //while(nrf24_isSending() && (t < 500)) {
    while(nrf24_isSending() && (t < 250)) {
      t++;
      HAL_Delay(1);  // this is not good, really need a microsecond timer
    }
    //    printf("(%u ms) ", t);
    status = nrf24_lastMessageStatus();
    if (status == NRF24_TRANSMISSON_OK) {
      printf("Sent ok. ");
    } else if (status == NRF24_MESSAGE_LOST) {
      printf("Message lost. :( ");
    }
    status = nrf24_retransmissionCount();
    printf("%u retransmissions.\r\n", status);

    /* See if the recipient sent back an ACK payload. */
    if(nrf24_dataReady()) {
      payload_length = nrf24_payloadLength();
      if (payload_length > 0) {
	printf("Got ACK payload (%u chars): ", payload_length);
	nrf24_getData(ackpl, payload_length);
	for(i=0; i<payload_length; i++) printf("%c", isprint(ackpl[i])?ackpl[i]:'.');
	for(i=0; i<payload_length; i++) printf(" %02x", ackpl[i]);
	printf("\r\n");
      }
    }
    nrf24_powerDown();
    HAL_Delay(1000);
  }
}

void nrf_rx(void)
{
  uint8_t i, n=0, payload_length;
  char rxbuf[32];
  char ackplbuf[5];
  
  nrf24_init();
  my_config();
  nrf24_tx_address(tx_address);
  nrf24_rx_address(rx_address, RX_ADDR_P0);

  while (1) {
    if (VCP_read(&i, 1) == 1) {
      nrf24_deinit(); // set SPI back to normal speed, etc
      return;
    }
    if (nrf24_dataReady()) {
      payload_length = nrf24_payloadLength();
      snprintf(ackplbuf, sizeof(ackplbuf), "R%03u", n++);
      nrf24_set_ack_payload(0, ackplbuf, 4);
      memset(rxbuf, 0, 16);
      nrf24_getData(rxbuf, payload_length);
      printf("Received %u bytes: ", payload_length);
      for(i=0; i<payload_length; i++)
	printf("%02x ", rxbuf[i]);
      for(i=0; i<payload_length; i++)
      	printf("%c", isprint(rxbuf[i])?rxbuf[i]:'.');
      printf("\r\n");
    }
  }
}
#endif


