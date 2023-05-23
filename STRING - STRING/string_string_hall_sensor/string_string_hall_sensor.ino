#include <OSCMessage.h>
#include <SLIPEncodedSerial.h>

/*
  Hall Sensor is connected to A0 of Arduino Nano
  For Arduino Nano clone board: set Processor to "ATmega328P (Old Bootloader)" in Tools/Processor
*/

SLIPEncodedSerial SLIPSerial(Serial);

const int hallSensor = A0;
const bool plotSerial = false; // debug

void setup() {
  if (plotSerial) {
    SLIPSerial.begin(9600);
  } else {
    // Maximum baudrate in Max seems to be 1843200 bps
    SLIPSerial.begin(1843200);
  }
  pinMode(hallSensor, INPUT);
}

void loop() {
  int val = analogRead(hallSensor);

  float voltage = val / 1023.0;   // 10 bit
  voltage = voltage * 2.0 - 1.0;  // (-1, 1)

  if (plotSerial) {
    SLIPSerial.print(-1);
    SLIPSerial.print(' ');
    SLIPSerial.print(1);
    SLIPSerial.print(' ');
    SLIPSerial.println(voltage);
  } 
  
  // Send OSC over Serial
  else {
    OSCMessage msg("/hall");
    msg.add(voltage);
    SLIPSerial.beginPacket();
    msg.send(SLIPSerial);
    SLIPSerial.endPacket();
    msg.empty();
  }
}
