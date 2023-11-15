# STM32_I2CwIRQDriver

Bare metal non blocking (!) master driver for I2C in STM32_L0x3.

## General description
I implied in my regular I2C driver project that it is possible to make a better driver using IRQs and thus avoid blocking "delay" commands in the code. Well, that turned out to be more complex than I initially estimated, so here we go...

First and foremost, why exactly do we want to have any work done on the original I2C master driver if it is working fine? Well, it does for a certain scenario where we aren’t constrained by timing whatsoever and could afford to “lose” some time until we have our I2C driver finish with its thing. If we are timing sensitive because, say, we want our entire (!) code to conclude in less than a few milliseconds, losing a few hundred microseconds doing nothing may not be acceptable. The issue is even more pressing in case we want an interrupt to trigger an immediate I2C transmit from our device: resting in the IRQ for an extended period of time is unanimously considered to be a very bad practice (as discussed in the DMA project, IRQs completely “phase out” the main code, meaning that it becomes completely deaf and blind to any activity outside the IRQ handler).

Obvious solution is to use DMA on the I2C as a cheat-code, though I am not exactly favour of wasting limited advanced resources on a problem that just isn’t “that” complex. I personally am using DMA only when I have no other choice or when the data flow is too great to be managed by alternative means. We will be using the BMP280 sensor again here and that one only sends 3 bytes over every time we sample the sensor.

Second obvious solution is to use a timer attached to the I2C that will replace our latency delay with an IRQ. This would allow us to do other things while we are “waiting”. I chose not to go this way simply because the I2C is already packed to the brim with IRQs. I figured there had to be a way to do all this without using a timer for it.

Of note, we use the BMP280 again and we need an oscilloscope too to check the I2C bus out.

### Problems…problems…
So, with the general what-the-hell-are-we-doing-here out of the way, what are the problems that are coming right at us?
1)	Flag and IRQ generation: Observing how the IRQs are triggered in the I2C peripheral, I only have one word of it: PAIN! They aren't generated when something is acknowledged but when the Tx and Rx buffers are in a certain state or when we have start and stop bits on the bus. This is not convenient at all since it impedes our capacity to daisy-chain activities when the timing is right. Similarly, we don’t have a flag when an ACK is generated or received but only when an ACK is missed, that is, when we have a NACK. (Ideal timing would be to have something always after an ACK to keep on loading the driver.)

2)	Overruns and data loss: If we remove any blocking delays, we may be executing our driver code again even before we have executed it a previous time. It can be a major problem if we execute our driver faster than the bus can react since we have an overruns where an existing Tx buffer state might be replaced;, we have data loss where an empty Rx buffer is captured and we can corrupt the ongoing I2C run by modifying the drive setup. Just for reference, we have our I2C driver running in fast mode at 400 kHz while our L0x3 can clock a hundred times faster at 32 MHz.

3)	Tx and Rx timing
When we want to do a readout, the process is to send the desired registers value as Tx to the sensor which is then followed by an Rx. Imagine that we want to read out a whole bunch of registers that aren’t neighbouring each other (so we can’t let the driver automatically increase the address): in this case, we absolutely must execute our driver in pairs with a Tx followed by a Rx, or we risk reading out the wrong values.

+1) DMA and TIM
As mentioned above, no DMA, no TIM for us this time.

### What do we want then?
To summarize, what we want is a recurringly executed I2C coms block that evolves itself gradually only when the bus is available. We don’t want blocking delays in there, no DMA and no TIM. By all intents and purposes, it should be fire-and-forget.

## Previous relevant projects
The following projects should be checked:
- STM32_I2CDriver (we will be modifying this original code)
- STM32_DMADriver (the IRQ part)

## To read
No additional reading material necessary.

## Particularities
We are still using the AUTOEND to generate a STOP bit at the end of each transmission. This means that the TC and TCR flags are not available to us to detect “transmission complete”. Instead, what we are using is the STOP bit detection IRQ on the bus to understand when our message is done. Mind, we only activate this IRQ within the readout function so as to allow us a dynamic transition from Tx to Rx. For simple Tx, it is simply not necessary…AUTOEND will close the bus for us. And for simple Rx…we don’t have a simple Rx since we are in master mode (we only have Tx and we have readout).

In the readout function, we have a blocking “while” loop which is not particularly elegant: we will keep on executing the loop – and not allow any other action to occur – until we have all data read out. We technically put our non-blocking I2C driver into a blocking framing. The reason for this is that I didn’t quite see an alternative over how we may control a sequence of readouts in any other way (not having a blocking loop would mean that we won’t be reading out data in appropriate pairs, that is, register 1 will not be replaced by data 1 in our message matrix). Since the point of the project was to have a non-blocking I2C driver and not a non-blocking readout function, I still consider this to be an acceptable solution. Mind, if we want to have only one readout, we can remove the while loop.

Of note, it is possible to not use the readout and just use the non-blocking Tx followed by a non-blocking Rx with the STOP bit activated (copy-paste what is in the while loop into the code), albeit in this case it must be ensured that the main loop will keep on executing – and only execute these two, that is, no additional Tx commands are planned - I2C commands on the same slave. The code can not tell the difference between one Tx and another, so a stray Tx could activate the Rx part or a missing Rx function could completely block the I2C bus.

At any rate, the recommendation is to NOT execute these non-blocking I2C functions close enough to each other.

Putting an oscilloscope probe on the I2C bus adds a lot of noise. This will make the bus less reliable and the captured data potentially corrupted. 

## User guide
As discussed above, we have an I2C driver that has no hard delays and works in a pretty much fire-and-forget manner. Once the Tx or the Rx function is called, it will be gradually and fully executed whenever the cpu is around to do it and the bus is free. The idea is that the main loop executes these functions whenever it is convenient.

In the code, we have a Tx test section that should be sniffed out using the oscilloscope. Tx test shows that we have a long data transfer ongoing but the printf command does not interrupt it (we do the I2C coms while we are doing the printf). On the scope, we should see the BMP280's address sent over, followed by a counter from 0x1 to 0x5.

Afterwards, we have the BMP280 readout identical to the original I2C project, but this time done using the new non-blocking Tx and Rx functions. Mind, the readout function is blocking though, so the example is merely to show that things work well.
