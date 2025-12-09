# ADXL375 accelerometer driver
The BMY ADXL375 driver is written in C++ 20. It's templated to support any SPI, iohandler, timer, interrupt that implements expected functions. Therefore this driver can be used in your Arduino, STM32, ... based projects.

## About ADXL375

The ADXL375 is a digital accelerometer that supports both SPI and I2C mode. It has a configurable data rate (100 Hz to 3200 Hz), an acceleration range (+/-200g) with 14-bit resoulution, a configurable FIFO (up to 32 data), and more features.

More information on the ADXL375 can be found in the datasheet: http://www.analog.com/static/imported-files/data_sheets/ADXL375.pdf
## Driver operation
Because the ADXL375 with I2C mode is limited to 800 Hz, the BMY ADXL375 driver uses 4 iohandlers SPI mode (mode 3) to anable higher data rates (up to 3200 Hz). When instantiated, SPI, iohandler, timer and interrupt handlers must be provided in the constructor. Then to initialize measurements, chip select pin, interrupt pin, data rate, and SPI clock rate must be provided in init function.

### SPI implementation
The user must provide a SPI implementation to handle SPI communication.                                                                                                                                                                     
The SPI class must implement :
```
  void begin() : to initialize SPI.
  void beginTransaction(uint32_t clock_speed, spi::BitOrder bit_order, spi::Mode spi_mode) : to start the transaction.
  void transfer(void *buf, size_t count) : transfer data. Note that SPI communication.
  void endTransaction(): end transaction.
```
If the user SPI API doesnt provide these implementations, an adapter can be used.

### Iohandler implementation
The user must provide a IOHANDLER implementation to set pin and read from pin.                                                                                                                                                                   
The IOHANDLER class must implement:
```
  void mode(uint8_t pin, iohandler::PinMode mode) : to set pin mode.
  void write(uint8_t pin, iohandler::PinStatus status): to do a digital write to the corresponding pin.
  iohandler::PinStatus read(uint8_t pin): read the value of the pin.
```
If the user Iohandler API doesnt provide these implementations, an adapter can be used.

### Timer implementation
The user must provide a TIMER implementation to handle delay.                                                                                                                                                                            
The TIMER class must implement:
```
  void delay_ms(uint32_t t): delay milliseconds.
  void delay_us(uint32_t t)
```
If the user Timer API doesnt provide these implementations, an adapter can be used.

### Interrupt implementation
The user must provide an INTERRUPT implementation to handle interrupts.                                                                                                                                                                     
The INTERRUPT must implement:
```
  void attachInterrupt(uint8_t interruptNum, void (*)(void *) userFunc, iohandler::PinStatus mode, void *param)
```
If the user Interrupt API doesnt provide these implementations, an adapter can be used.

  
