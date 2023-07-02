/*
  Two Hall Sensors connected to A0 and A1 of Arduino Nano
  For Arduino Nano clone board: set Processor to "ATmega328P (Old Bootloader)" in Tools/Processor
*/

#include <OSCMessage.h>
#include <SLIPEncodedSerial.h>

SLIPEncodedSerial SLIPSerial(Serial);

const int hallSensor1 = A0;
const int hallSensor2 = A1;

const bool plotSerial = false;

void setup() {
  if (plotSerial) {
    SLIPSerial.begin(9600);
  } else {
    // Maximum baudrate in Max seems to be 1843200 bps
    SLIPSerial.begin(1843200);
  }
  pinMode(hallSensor1, INPUT);
  pinMode(hallSensor2, INPUT);
}

void loop() {
  int v1 = analogRead(hallSensor1);
  int v2 = analogRead(hallSensor2);

  float hall1 = v1 / 1023.0;  // 10 bit
  hall1 = hall1 * 2.0 - 1.0;  // (-1, 1)

  float hall2 = v2 / 1023.0;
  hall2 = hall2 * 2.0 - 1.0;

  if (plotSerial) {
    SLIPSerial.print(-1);
    SLIPSerial.print(' ');
    SLIPSerial.print(1);
    SLIPSerial.print(' ');
    SLIPSerial.print(hall1);
    SLIPSerial.print(' ');
    SLIPSerial.println(hall2);
  }

  // Send OSC over Serial
  else {
    OSCMessage msg("/hall");
    msg.add(hall1).add(hall2);

    // Send sample frequency 
    // msg.add(getSampleFrequency());

    SLIPSerial.beginPacket();
    msg.send(SLIPSerial);
    SLIPSerial.endPacket();
    msg.empty();
  }
}

float getSampleFrequency() {
  static unsigned long prevMicros = micros();

  unsigned long dt = micros() - prevMicros;
  float sampleFreq = 1.0 / (dt / 1.0e6);
  prevMicros = micros();

  return sampleFreq;
}
