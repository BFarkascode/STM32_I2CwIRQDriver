/*
 *
 *  Created on: 14 Nov 2023
 *  Project: STM32_I2CwIRQDriver
 *  File: I2CwIRQDriver_STM32L0x3.c
 *  Author: BalazsFarkas
 *  Processor: STM32L053R8
 *  Compiler: ARM-GCC (STM32 IDE)
 *  Program version: 1.0
 *  Program description: N/A
 *  Hardware description/pin distribution: N/A
 *  Modified from: I2CDriver_STM32L0x3.c
 *  Change history: N/A
 *
 */


/*
 * Based on the original (blocking) I2CDriver_STM32L0x3 source code.
 * Completely reworked driver to run on IRQs instead of blocking delays.
 * All delay functions are removed.
 *
 */

#include "I2CwIRQDriver_STM32L0x3.h"
#include "ClockDriver_STM32L0x3.h"
#include "stm32l053xx.h"


//1) Initialization
void I2CConfig(uint8_t dev_own_addr) {

	/*
	 * What is different here?
	 *
	 * Compared to our original setup, we enable the TXE, the RXNE and the NACK IRQs. We DON'T enable the STOP IRQ, that will be done so locally in the readout.
	 * Mind, stop generation can be ignored when we do Tx since we are in master mode using AUTOEND. AUTOEND disables the TC flag.
	 *
	 * TXE is generated when the Tx buffer is empty. The address is not stored in the Tx buffer despite taking up bus time.
	 * RXNE is generated when the Rx buffer is NOT empty (so on the 8th bit, just before master ACK to the slave - and, potentially, the stop bit).
	 * NACK when master mode is only relevant while we scan for the slave. In any other cases, NACK from a slave that has been found before means an I2C error.
	 *
	 *
	 */

	//1)Enable clocking in the RCC, set I2CCLK source clock, enable GPIO clocking - PB8 SCL, PB9 SDA

	RCC->APB1ENR |= (1<<21);															//enable I2C1 clock
	RCC->CCIPR &= ~(1<<12);																//APB1 is selected as I2C source clock
	RCC->CCIPR &= ~(1<<13);																//APB1 is selected as I2C source clock
	RCC->IOPENR |=	(1<<1);																//PORTB clocking

	//2)Set GPIO parameters (mode, speed, pullup) - PB8 SCL, PB9 SDA
	GPIOB->MODER &= ~(1<<16);															//alternate function for PB8
	GPIOB->MODER &= ~(1<<18);															//alternate function for PB9
																						//Note: MODER resets to 0xFFFF, so we need to write 0s instead of 1s
	GPIOB->OTYPER |= (1<<8);															//open drain for PB8
	GPIOB->OTYPER |= (1<<9);															//open drain for PB9
	GPIOB->OSPEEDR |= (3<<16);															//high speed PB8
	GPIOB->OSPEEDR |= (3<<18);															//high speed PB9
	GPIOB->PUPDR |= (1<<16);															//pullup PB8
	GPIOB->PUPDR |= (1<<18);															//pullup PB9

	//Note: AFR values are in the device datasheet for L0. For I2C1, they will be AF4 as seen on page 45 of the datasheet.
	GPIOB->AFR[1] |= (4<<0);															//PB8 AF4 setup
	GPIOB->AFR[1] |= (4<<4);															//PB9 AF4 setup

	//3)Set a clock source for the internal clock of the I2C
	I2C1->CR1 &= ~(1<<0);																//disable I2C, used as software reset here - no designated SWRST bit in registers
	Delay_us(1);																		//PE should be LOW for at least 3 cycles to take effect! 3 cycles at 8 MHz is 375 ns.
																						//config registers are not impacted by the PE reset, only START, STOP, NACK, and various ISR registers

	//4) Set timing - standard mode selected with 8 MHz I2CCLK

	//Standard timing
//	I2C1->TIMINGR |= (19<<0);															//SCLL value shall be 0x13 for 8 MHz I2CCLK, Standard mode (see page 727, ref manual)
//	I2C1->TIMINGR |= (15<<8);															//SCLH value shall be 0xF for 8 MHz I2CCLK, Standard mode
//	I2C1->TIMINGR |= (2<<16);															//SDADEL value shall be 0x2 for 8 MHz I2CCLK, Standard mode
//	I2C1->TIMINGR |= (4<<20);															//SCLDEL value shall be 0x4 for 8 MHz I2CCLK, Standard mode
//	I2C1->TIMINGR |= (1<<28);															//PRESC value shall be 0x1 for 8 MHz I2CCLK, Standard mode

	//Fast timing
	I2C1->TIMINGR |= (9<<0);															//SCLL value shall be 0x9 for 8 MHz I2CCLK, Fast mode (see page 727, ref manual)
	I2C1->TIMINGR |= (3<<8);															//SCLH value shall be 0x3 for 8 MHz I2CCLK, Fast mode
	I2C1->TIMINGR |= (1<<16);															//SDADEL value shall be 0x1 for 8 MHz I2CCLK, Fast mode
	I2C1->TIMINGR |= (3<<20);															//SCLDEL value shall be 0x3 for 8 MHz I2CCLK, Fast mode
	I2C1->TIMINGR &= ~(15<<28);															//PRESC value shall be 0x0 for 8 MHz I2CCLK, Fast mode

	//5) Set own address
	I2C1->OAR1 &= ~(1<<15);																//we disable the own address
	I2C1->OAR1 &= ~(1<<10);																//we choose 7-bit won address
	I2C1->OAR1 |= (dev_own_addr<<1);													//we define the 7-bit own address
	I2C1->OAR1 |= (1<<15);																//we enable the own address

	//6)Clean up the ISR register
	I2C1->ISR |= (1<<0);																//we flush the transmit register

	//7)Set IRQ enablers
//	I2C1->CR1 |= (1<<6);																//TC interrupt enabled on the TC flag - (1<<6) ISR
//	I2C1->CR1 |= (1<<5);																//Stop detection flag - (1<<5) ISR
	I2C1->CR1 |= (1<<1);																//TXIS/TXE interrupt enabled on the ? flag  - (1<<1) ISR, set when the TX buffer is empty
																							//Note: TXE is on (1<<0) ISR
	I2C1->CR1 |= (1<<2);																//RXNE interrupt enabled on the ? flag - (1<<2) ISR, set when the RX buffer is full
	I2C1->CR1 |= (1<<4);																//NACK interrupt enabled on the ? flag - (1<<4) ISR

	//8)Enable I2C
	I2C1->CR1 &= ~(1<<12);																//analog filter enabled
	I2C1->CR1 &= ~(1<<17);																//clock stretch enabled
																						//this must be kept as such for MASTER mode
	I2C1->CR1 |= (1<<0);																//enable I2C
	Delay_us(1);																		//We wait for the setup to take effect
}

//2) Scanning
void I2CSCANNER (uint8_t slave_addr) {
	/*
	 * What is different here?
	 *
	 * We have a scanning_bus flag to indicate, which action is active on the bus.
	 * We discard all the code after we started the scanning. Stopping the process is now done using IRQs.
	 *
	 */

	scanning_bus = Yes;

	//1)We reset the transmission control register
	I2C1->CR2 = 0x0;																	//we reset the CR2 register and rebuild it completely to avoid an address being stuck in there

	//2)We set the slave address and the addressing mode
	I2C1->CR2 = (slave_addr << 1);
	I2C1->CR2 &= ~(1<<11);																//7 bits

	//3)We set NBYTES as 0
	I2C1->CR2 |= (0 << 16);																//NBYTES does not include the slave address

	//4)We want an AUTOEND and no RELOAD
	I2C1->CR2 |= (1 << 25);
	I2C1->CR2 &= ~(1 << 24);

	//5)We enable PE and set the START bit
	I2C1->CR2 &= ~(1<<10);																//we write to the slave

	I2C1->CR1 |= (1<<0);																//enable

	I2C1->CR2 |= (1<<13);																//start
}

//3) Transmission
void I2CTX (uint8_t slave_addr) {
	/**
	 *
	 * What is different here?
	 *
	 * We have an "is the bus busy?" control at the start of the function. If the bus is busy, we completely skip the function. This is to avoid overruns.
	 * We have a Tx_ongoing flag to indicate, which action is active on the bus.
	 * We have a similar flag for Rx that is negated.
	 * We discard all the code after we started the scanning. Stopping the process - and evolving it, if necessary - is now done using IRQs
	 *
	 * **/

	if((I2C1->ISR & (1<<15)) == (1<<15)) {

		//do nothing
		//Note: we skip the Tx if the bus is busy

	} else {

	//1)We reset the transmission control register
	I2C1->CR2 = 0x0;																	//we reset the CR2 register and rebuild it completely to avoid an address being stuck in there

	//2)We set the slave address and the addressing mode
	I2C1->CR2 = (slave_addr << 1);														//we write the slave address to the SADD register
																						//Note: we are in 7-bit address mode, as it was set above in the I2C config
	I2C1->CR2 &= ~(1<<11);																//7 bits

	//3)We set NBYTES
	I2C1->CR2 |= (Tx_number_of_bytes << 16);

	//4)AUTOEND with write as direction, no RELOAD
	I2C1->CR2 |= (1 << 25);
	I2C1->CR2 &= ~(1 << 24);
	I2C1->CR2 &= ~(1<<10);

	//5)We enable PE and set the START bit

	I2C1->CR1 |= (1<<0);																//enable

	Tx_ongoing = Yes;
	Rx_ongoing = No;

	I2C1->CR2 |= (1<<13);																//start

	}
}


//4) Reception
void I2CRX (uint8_t slave_addr, uint8_t RX_number_of_bytes) {
	/*
	 *
	 * What is different here?
	 *
	 * We have an "is the bus busy?" control at the start of the function. If the bus is busy, we completely skip the function. This is to avoid overruns.
	 * We have a Rx_ongoing flag to indicate, which action is active on the bus.
	 * We have a similar flag for Tx that is negated.
	 * We discard all the code after we started the scanning. Stopping the process - and evolving it, if necessary - is now done using IRQs
	 *
	 */

	if((I2C1->ISR & (1<<15)) == (1<<15)) {

		//Do nothing. We skip the Rx.
//		printf("I2C Rx error. Bus busy \r\n");

	} else {

		//1)We reset the transmission control register
		I2C1->CR2 = 0x0;																//we reset the CR2 register and rebuild it completely to avoid an address being stuck in there

		//2)We set the slave address and the addressing mode
		I2C1->CR2 = (slave_addr << 1);													//we write the slave address to the SADD register
																						//Note: we are in 7-bit address mode, as it was set above in the I2C config
		I2C1->CR2 &= ~(1<<11);															//7 bits

		//3)We set NBYTES
		I2C1->CR2 |= (RX_number_of_bytes << 16);

		//4)AUTOEND with write as direction, no RELOAD
		I2C1->CR2 |= (1 << 25);
		I2C1->CR2 &= ~(1 << 24);

		//5)We read now!
		I2C1->CR2 |= (1<<10);

		//6)We enable PE and set the START bit

		I2C1->CR1 |= (1<<0);															//enable

		Tx_ongoing = No;
		Rx_ongoing = Yes;

		I2C1->CR2 |= (1<<13);															//start

	}

}

//5) Readout
//The following function takes an array of registers to read out and then replaces the registers with their readout values

void I2CReadout_OG(uint8_t slave_addr, uint8_t number_of_readouts, uint8_t* data_buffer) {
	/*
	 * What happens here?
	 * We write and then read out, using the same array.
	 * Be aware that a Tx needs to be used to tell the slave that we want to read and from which register.
	 * Then, we put out driver into Rx mode, send over the address and await how the sensor is automatically starting to send over the data.
	 *
	 * */

	for (int i = 0; i < number_of_readouts; i++) {
		if (Rx_finished == 1){

			bytes_to_send_ptr = &data_buffer[i];
			Tx_number_of_bytes = 1;
			I2CTX(slave_addr);
			Delay_us(55);

			//results show that if the delay above falls below 30, the IC2TX function can not loop

		} else {
			//do nothing
		}

//		I2CTX_OG(slave_addr, 1, &data_buffer[i]);
		if (Tx_finished == Yes){

			bytes_received_ptr = &data_buffer[i];
			I2CRX(slave_addr, 1);
			Delay_us(55);																//the problem is with the looping of the I2C readout. If we do it this way. we may execute it once without an IRQ activating, effectively skipping one element.
																						//the solution is to execute a Tx side only when there is
																						//the problem is also if the TX and RX fire-and-forget functions slide on each other. We must not write on the bus before the first address is sent.
		} else {
			//do nothing
		}

		//if we have a Tx error, we should recall the Readout with the values that were erroreous
		//if we have a Rx error, we should recall the Readout and skip Tx until

	}

}


void I2CReadout(uint8_t slave_addr, uint8_t number_of_readouts, uint8_t* data_buffer) {
	/*
	 * What is different here?
	 *
	 * First we start by activating the STOP IRQ. This is important since the STOP bit detection is used - in lieu of the TC which is not active with AUTOEND - to detect the end of a message.
	 * We have a "while" loop with a decreasing number instead of a "for" loop. The decreasing number is decreased in the IRQ when both Tx and Rx have finished.
	 * The "while" loop contains a state machine, governed by where we are in our communication. Rx can only occur after a Tx, ensuring a readout.
	 * Due to how the new Tx and Rx functions are defined, executing them multiple times after each other will only take effect if the bus is not busy. Thus, the "while" loop can execute infinite times without corrupting the bus.
	 * We only allow progress when both Tx and Rx are done.
	 * We remove the STOP IRQ which should not be used ourside the readout function.
	 *
	 * */

	//we enable the STOP IRQ
	I2C1->CR1 |= (1<<5);																//Stop detection flag - (1<<5) ISR

	while (number_of_readouts) {														//readout is blocking!

		  I2C1->CR1 |= (1<<5);															//we use the STOP IRQ to detect the end of a transmission in lieu of the TC flag (which is deactivated using AUTOEND)
		  																				//Mind, the stop generation and the final RXNE could overlap, so in the IRQ stop testing should come before RXNE testing

		  if((Tx_finished == No) & (Rx_finished == No)) {

			bytes_to_send_ptr = &data_buffer[(number_of_readouts-1)];
			Tx_number_of_bytes = 1;
			I2CTX(slave_addr);

		  } else if ((Tx_finished == Yes) & (Rx_finished == No)){

			bytes_received_ptr = &data_buffer[(number_of_readouts-1)];
			I2CRX(slave_addr, 1);

		  } else if ((Tx_finished == Yes) & (Rx_finished == Yes)){

			  Tx_finished = No;
			  Rx_finished = No;
			  number_of_readouts--;
			  I2C1->CR1 &= ~(1<<5);														//we remove the STOP IRQ trigger.
			  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	//Failing to do so likely freezes the bus due to noise. (Fake stop bit detected on the bus)
			  	  	  	  	  	  	  	  	  	  	  	  	  	  	  					//Note: adding a scope probe on the I2C increase noise drastically!

		  } else {

			  Tx_finished = No;
			  Rx_finished = No;

		  }

	}

}


//6)Handler
void I2C1_IRQHandler (void) {

	/*
	 * What is different here?
	 *
	 * We had no IRQ for the I2C beforehand.
	 * Now, we have multiple triggers, such as:
	 * 1)NACK - only used when scanning. We simply reset the peripheral when we had a NACK
	 * 2)TXE - we use the Tx number of bytes to either update our Tx buffer with the new value, or, in case nbytes have been reached, to shut off the coms. We don't need to generate a STOP bit, that is done using AUTOEND.
	 * 3)RXNE - we extract data when it is available
	 * 4)STOP - we manage the end of message progression when using the readout function.
	 * Note: STOP IRQ is only active during readout!
	 * Note: While TXE IRQ shuts off the peripheral once nbytes are done, the RXNE only shuts off during readout. (Theoretically, it is not necessary to shut off the peripheral whatsoever.)
	 *
	 *
	 * Of note, it is possible that a STOP IRQ and an RXNE overlaps. In this case, the handler will be called twice, once for RXNE, once for the STOP bit. This was chosen to avoid data loss.
	 *
	 * */


	if ((I2C1->ISR & (1<<4)) == (1<<4)) {												//if we had the IRQ on NACK, it means the I2C com can stop since it means that we didn't find the slave
																						//Note: since we are the master here, the only time we can have a NACK is when the slave is not found or a TX is not acknowledged

		if(scanning_bus == Yes) {															//we check if we had been scanning the bus or not

			scan_reply = 0;																//the init function returns "0" while the NACKF bit is 1
			scanning_bus = No;
			I2C1->ICR |= (1<<4);														//we clear the flag
			I2C1->CR1 &= ~(1<<0);														//we turn off PE and reset the I2C bus
			Delay_us(1);																//we need to wait 3 APB cycles after PE is cleared. We wait for 1 us instead.

		} else if (scanning_bus == No){

			printf("I2C1 NACK error \r\n");												//If we have a NACK outside scanning, it means we have a coms error
			while(1);

		} else {

			//do nothing

		}

	} else if ((I2C1->ISR & (1<<1)) == (1<<1)) {										//If we had TXE trigger the IRQ
																						//Note: unlike the Rx side, the TXE does not overlap with the STOP bit

		if (Tx_number_of_bytes > 0) {

				I2C1->TXDR = (volatile uint8_t) *bytes_to_send_ptr++;					//we load the byte and thus will have TXIS LOW until the data byte is cleared
				Tx_number_of_bytes--;

		} else {

			I2C1->CR1 &= ~(1<<0);														//we turn off PE and reset the I2C bus
			Delay_us(1);																//we need to wait 3 APB cycles after PE is cleared. We wait for 1 us instead.

		}

	} else if ((I2C1->ISR & (1<<2)) == (1<<2)) {										//if we have RXNE trigger the IRQ

		*bytes_received_ptr = I2C1->RXDR;												//we dereference the pointer, thus we can give it a value
																						//in other words, we read out the Rx buffer and thus reset the flags
		bytes_received_ptr++;															//this is technically an address. A pointer is technically a memory address. Here we step through the array at the address.

	} else if ((I2C1->ISR & (1<<5)) == (1<<5)) {										//Stop IRQ
																						//we use the stop detection IRQ to remove all bus state flags
																						//Note: we use the bus state flags to schedule one state after another (Tx after Rx)
																						//Note: the final RXNE and the STOP may overlap, thus stop testing should occur BEFORE the RXNE testing, otherwise we might have the two flags high at the same time
																						//Note: STOP bit may become activated while we have the IRQ activated using RXNE
																						//Note: since we don't clear the STOP IRQ with the RXNE, we should just re-trigger the IRQ once it has passed

		if (Tx_ongoing == Yes) {															//if this was a Tx stop

			Tx_ongoing = No;
			Tx_finished = Yes;															//we just set the flags

		} else if (Rx_ongoing == 1) {													//if this was an Rx stop

			Rx_ongoing = No;
			Rx_finished = Yes;															//we set the finished flag
			I2C1->CR1 &= ~(1<<0);														//we turn off PE and reset the I2C bus
			Delay_us(1);																//we need to wait 3 APB cycles after PE is cleared. We wait for 1 us instead.

		} else {
			//do nothing
		}

		I2C1->ICR |= (1<<5);															//we clear the flag																						//Note: within an Rx stop, we have the STOP IRQ and the last RXNE overlapping

	} else {

		//do nothing

	}

}


//7)Priority and enable
void I2C1IRQPriorEnable(void) {
	/*
	 * We call the two special CMSIS functions to set up/enable the IRQ.
	 *
	 * */
	NVIC_SetPriority(I2C1_IRQn, 1);														//IRQ priority for I2C1
	NVIC_EnableIRQ(I2C1_IRQn);															//IRQ enable for I2C1
}
