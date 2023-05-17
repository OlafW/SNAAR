#include "Wire.h"
#include "WiFi.h"
#include <esp_wifi.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <MPU6050_light.h>
#include "networksettings.h"

//------------------------------------Settings-----------------------------------------------//

// Smaller range = higher sensitivity
enum GyroRanges { GYRO_250,
                  GYRO_500,
                  GYRO_1000,
                  GYRO_2000 };  // Gyroscope range (+- degrees / second)
enum AccelRanges { ACCEL_2G,
                   ACCEL_4G,
                   ACCEL_8G,
                   ACCEL_16G };  // Accelerometer range (+- times gravity (9.81 m/s^2))

int gyroRange = GYRO_250;
int accelRange = ACCEL_2G;
const boolean mpuIsUpsideDown = false;

//---------------------------------------------------------------------------------------------//

WiFiUDP Udp;
MPU6050 mpu(Wire);

int mpuID;
OSCMessage outMessage;

unsigned long prevMillis = 0;
unsigned int receiveOSCinterval = 500;  // ms

void setup() {
  Serial.begin(921600);
  Wire.begin();
  delay(2000);
  Serial.println("Starting...");
  
  // Set mpu ID based on mac address
  for (int i = 0; i < 2; i++) {
    if (ESP.getEfuseMac() == esp_mac[i]) {
      mpuID = i + 1;
      // Set OSC address of outgoing messages
      String OSC_address = ("/mpu/" + String(mpuID));
      outMessage.setAddress(OSC_address.c_str());
    }
  }
  Serial.print("MPU ID is: ");
  Serial.println(mpuID);

  // Wifi setup
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Trying to connect to WiFi...");
    delay(1000);
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Local port: ");
  Serial.println(localPort);

  // UDP setup
  Serial.println("\nStarting UDP");
  Udp.begin(localPort);

  // MPU6050 setup
  Serial.println("\nSetting up MPU6050");
  byte status = mpu.begin();
  Serial.print("status: ");
  Serial.println(status);

  while (status != 0) {
    Serial.print("MPU setup failed");
    Serial.print(" - error code: ");
    Serial.println(status);
    delay(1000);
  }

  // MPU config
  mpu.upsideDownMounting = mpuIsUpsideDown;
  mpu.setGyroConfig(gyroRange);
  mpu.setAccConfig(accelRange);

  // MPU calibration
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);  // Calibrate gyro and accelerometer
  Serial.println("Done!\n");
}


void loop() {
  mpu.update();

  sendOSC();
  if (millis() - prevMillis > receiveOSCinterval) {
    receiveOSC();
    prevMillis = millis();
  }
}


void sendOSC() {
  // Send gyro magnitude and angle x, y
  outMessage.add(getGyroMag());
  outMessage.add(mpu.getAngleX()).add(mpu.getAngleY()).add(mpu.getAngleZ());

  Udp.beginPacket(outIP, outPort);
  outMessage.send(Udp);
  Udp.endPacket();
  outMessage.empty();
}


void receiveOSC() {
  OSCMessage msg;
  int msgSize = Udp.parsePacket();

  if (msgSize > 0) {
    while (msgSize--) {
      msg.fill(Udp.read());
    }

    // Calibrate MPU
    if (!msg.getError()) {
      msg.dispatch("/resetMPU", resetMPU);
    } else {
      Serial.print("OSC error: ");
      Serial.println(msg.getError());
    }
    msg.empty();
  }
}

float getGyroMag() {
  const static float abs_norm = sqrt(3);

  float gyroMag = sqrt(mpu.getGyroX() * mpu.getGyroX() + mpu.getGyroY() * mpu.getGyroY() + mpu.getGyroZ() * mpu.getGyroZ());

  if (gyroRange == GYRO_250) gyroMag /= 250.0;
  else if (gyroRange == GYRO_500) gyroMag /= 500.0;
  else if (gyroRange == GYRO_1000) gyroMag /= 1000.0;
  else if (gyroRange == GYRO_2000) gyroMag /= 2000.0;

  gyroMag /= abs_norm;  // normalize (0-1)

  return gyroMag;
}

float getAccelMag() {
  const static float abs_norm = sqrt(3);

  float accelMag = sqrt(mpu.getAccX() * mpu.getAccX() + mpu.getAccY() * mpu.getAccY() + mpu.getAccZ() * mpu.getAccZ());

  if (accelRange == ACCEL_2G) accelMag /= 2.0;
  else if (accelRange == ACCEL_4G) accelMag /= 4.0;
  else if (accelRange == ACCEL_8G) accelMag /= 8.0;
  else if (accelRange == ACCEL_16G) accelMag /= 16.0;

  accelMag /= abs_norm;

  return accelMag;
}

void sendAllData() {
  OSCBundle bundle;

  bundle.add("/mpu/accel").add(mpu.getAccX()).add(mpu.getAccY()).add(mpu.getAccZ());
  bundle.add("/mpu/gyro").add(mpu.getGyroX()).add(mpu.getGyroY()).add(mpu.getGyroZ());
  bundle.add("/mpu/angle").add(mpu.getAngleX()).add(mpu.getAngleY()).add(mpu.getAngleZ());

  Udp.beginPacket(outIP, outPort);
  bundle.send(Udp);
  Udp.endPacket();
  bundle.empty();
}

void resetMPU(OSCMessage& msg) {
  Serial.println("Resetting MPU");

  // Send back confirmation message
  String OSC_address = "/mpu/" + String(mpuID) + "/info";
  OSCMessage response(OSC_address.c_str());

  response.add("resetting MPU");
  Udp.beginPacket(outIP, outPort);
  response.send(Udp);
  Udp.endPacket();
  response.empty();

  gyroRange = msg.getInt(0);
  accelRange = msg.getInt(1);
  Serial.print("Gyroscope range: ");
  Serial.println(GyroRanges(gyroRange));
  Serial.print("Accelerometer range: ");
  Serial.println(AccelRanges(accelRange));

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  while (status != 0) {
    Serial.println("MPU setup failed");
    delay(1000);
  }
  mpu.upsideDownMounting = mpuIsUpsideDown;
  mpu.setGyroConfig(gyroRange);
  mpu.setAccConfig(accelRange);

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);  // Calibrate gyro and accelerometer
  Serial.println("Done!\n");
}

int scanI2CAddresses() {
  byte error, address;
  int nDevices = 0;

  Serial.println("\nScanning I2C addresses...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);

    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.println("I2C scanning done");
  }
  return nDevices;
}