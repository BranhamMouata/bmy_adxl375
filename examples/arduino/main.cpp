#include "arduino_interrupt.h"
#include "arduino_spi.h"
#include "arduino_timer.h"
#include "arduino_wire.h"
#include "bmy_adxl375.h"
#include <Arduino.h>
using namespace bmy;

ArduinoSpiAdapter arduino_spi{};
ArduinoWireAdapter arduino_wire{};
ArduinoTimerAdapter arduino_timer{};
ArduinoInterruptAdapter arduino_interupt{};
Adxl375 accelerometer(&arduino_spi, &arduino_wire, &arduino_timer, &arduino_interupt);
void setup(void) {
  Serial.begin(9600);
  // begin spi
  arduino_spi.begin();
  accelerometer.init(adxl375::kChipSelectPin, adxl375::kDataRate, adxl375::kInterPin);
}

void loop(void) {
  Serial.println("Fifo counts : ");
  Serial.println(accelerometer.data_count());
  delay(5);
}