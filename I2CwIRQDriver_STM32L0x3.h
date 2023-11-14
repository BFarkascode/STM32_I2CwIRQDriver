/*
 * I2CwIRQDriver_STM32L0x3.h			v.1.0
 *
 *  Created on: 14 Nov 2023
 *      Author: Balazs Farkas
 */

#ifndef INC_I2CDRIVER_CUSTOM_H_
#define INC_I2CDRIVER_CUSTOM_H_

#include "stdint.h"
#include "ClockDriver_STM32L0x3.h"
#include "main.h"

//LOCAL CONSTANT
//bus latency
static const uint8_t I2C_bus_latency_us = 0x1b;			//it takes, at minimum, 90 um (0x5A) for a byte and an ACK to cross the bus at 100 kHz
														//it takes, at minimum, 25 um (0x19) for a byte and an ACK to cross the bus at 400 kHz
														//Note: on startup, the actual latency will be more than the bare minimum since:
																//SCL needs time to clock in and generate a pulse
																//the start bit takes some time to generate as well
																//there is some stretching between the start condition and the byte being sent
//LOCAL VARIABLE
//static uint8_t* bytes_received_ptr;
static enum_Yes_No_Selector Tx_ongoing = No;
static enum_Yes_No_Selector Rx_ongoing = No;

//EXTERNAL VARIABLE
extern uint8_t scan_reply;
extern enum_Yes_No_Selector scanning_bus;
extern enum_Yes_No_Selector Tx_finished;
extern enum_Yes_No_Selector Rx_finished;
extern uint8_t Tx_number_of_bytes;
extern uint8_t* bytes_to_send_ptr;
extern uint8_t* bytes_received_ptr;

//FUNCTION PROTOTYPES
void I2CConfig(uint8_t dev_own_addr);
void I2CSCANNER (uint8_t slave_addr);
int I2CTX_OG (uint8_t slave_addr, uint8_t number_of_bytes, uint8_t *bytes_to_send);
void I2CTX (uint8_t slave_addr);
void I2CRX (uint8_t slave_addr, uint8_t Rx_number_of_bytes);
void I2CReadout(uint8_t slave_addr, uint8_t number_of_readouts, uint8_t* data_buffer);


#endif /* INC_I2CDRIVER_CUSTOM_H_ */
