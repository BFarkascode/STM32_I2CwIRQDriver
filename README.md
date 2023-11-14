# STM32_I2CwIRQDriver
Bare metal non blocking (!) master driver for I2C in STM32_L0x3.

We use the BMP280 again and we need an oscilloscope too to check the I2C bus out.

I implied in my regular I2C driver project that it is possible to make a better driver using IRQs and thus avoid blocking "delay" commands in the code. Well, that turned out to be more complex than I initally estimated, so here we go...

Problems
1)Do it without DMA or timers since those are valuable resources
The obvious solution is ofcoruse to attach a timer to the I2C and then whenever a certain latency time has passed, we do a thing. This is great as long as you don't need those timers somewhere else. Aslo, there are a plathora of IRQs and flags in the I2C, so it is bound to be possible to do this without using additional peripherals...that is to make a non-blocking I2C driver.

2)Flag and IRQ generation is not what is convenient
Flags are a PAIN in the I2C peripheral. They aren't generated when something is acknowledged but when the Tx and Rx buffers are in a certain state or when we have start and stop bits on the bus.


3)Overruns and data loss happens when we execute the next function before the previous one has finished
If we execute our driver faster than the bus can react - which is pretty easy - exisitng setup and data values will be updated until the bus becomes free again. This is very undesired, especially when we want to have a string of data read out.

4)Tx and Rx timing for readout demands that we have a Tx followed by an Rx
For a readout, we send first the address as Tx follwoed by an Rx.



	  //what we want is a recurringly executed I2C coms block that evolves itself gradually only when the bus is available
	  //generally, we can get around this by adding delays to the functions, albeit these will block the code
	  //we want an I2C coms block that can be executed infinite times, but will only act when the bus is empty (and in the case of readout, Tx and Rx are in a sequence)
	  //the problem is with executing such block faster than necessary, which can either corrupt the original I2C com or evolve the data readout unnecessarily
	  //we don't want delays in the code. We want the coms to occur whenever the code is executing it AND the bus is empty.
	  //scope shows that we have request and asnwer sent after each other, as expected



The result is an I2C driver that has no hard delays and works in a pretty much fire-and-forget manner. Once the function is called, it will be gradually and fully executed whenever the cpu is around to do it and the bus is free..
Another benefit is that it can daisy-chain Tx and Rx after each other without any delay between the two.

Also, putting a scope probe on the I2C bus adds a lot of noise!

BMP280 readout goes as before.

Tx test shows that we have a long data transfer but the printf command does not interrupt it (we do the I2C coms WHILE(!) we are doing the printf). On the scope, we should see the BMP280's address sent over, followed by a counter from 0x1 to 0x5.



