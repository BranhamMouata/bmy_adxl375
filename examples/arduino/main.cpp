#include "arduino_interrupt.h"
#include "arduino_iohandler.h"
#include "arduino_spi.h"
#include "arduino_timer.h"
#include "bmy_adxl375.h"
#include <Arduino.h>
using namespace bmy;

ArduinoSpiAdapter arduino_spi{};
ArduinoIoHandlerAdapter arduino_iohandler{};
ArduinoTimerAdapter arduino_timer{};
ArduinoInterruptAdapter arduino_interupt{};
Adxl375 accelerometer(&arduino_spi, &arduino_iohandler, &arduino_timer, &arduino_interupt);
void setup(void) {
  Serial.begin(9600);
  // begin spi
  arduino_spi.begin();
  accelerometer.init(adxl375::kChipSelectPin, adxl375::kInterPin, adxl375::kClkSpeed,
                     adxl375::kDataRate);
}

void loop(void) {
  Serial.println("Fifo counts : ");
  Serial.println(accelerometer.data_count());
  Serial.println("New data : ");
  Serial.println(accelerometer.new_data());
  delay(5);
}