#include "Wire.h"

#define DEVICE_I2C_ADDR   0x02

uint8_t deviceID = 0;
uint32_t encoderCountLH = 0;
uint32_t encoderCountRH = 0;
uint16_t rpmLH = 0;
uint16_t rpmRH = 0;
uint16_t encoderPPR = 0;

enum Register {
  EncoderID = 0x00,
  EncoderData = 0x01,
  EncoderPPR = 0x02
};

void setup() {
  // put your setup code here, to run once:
  //Config digital interfaces
  Serial.begin(9600);
  Wire.begin(DEVICE_I2C_ADDR);
  //Wire.onReceive(I2CRX_IRQ);
  //Wire.onRequest(I2CReq_IRQ);

  //Test communication and get base information from encoder
  //Begin transmission to Encoder
  Wire.beginTransmission(DEVICE_I2C_ADDR);
  //Ask for data
  Wire.write(EncoderID);
  //End transmission
  Wire.endTransmission();
  //Request bytes
  Wire.requestFrom(DEVICE_I2C_ADDR, 1);

  while(Wire.available() < 1);

  deviceID = Wire.read();

  //Begin transmission to Encoder
  Wire.beginTransmission(DEVICE_I2C_ADDR);
  //Ask for data
  Wire.write(EncoderPPR);
  //End transmission
  Wire.endTransmission();
  //Request bytes
  Wire.requestFrom(DEVICE_I2C_ADDR, 2);

  while(Wire.available() < 2);

  encoderPPR = Wire.read() << 8;
  encoderPPR += Wire.read();
}

uint32_t timestamp = 0;
void loop() {
  // put your main code here, to run repeatedly:
  if((timestamp + 100) < millis()) {
    //Begin transmission to Encoder
    Wire.beginTransmission(DEVICE_I2C_ADDR);
    //Ask for data
    Wire.write(EncoderData);
    //End transmission
    Wire.endTransmission();
    //Request bytes
    Wire.requestFrom(DEVICE_I2C_ADDR, 12);

    while(Wire.available() < 12);

    encoderCountLH = Wire.read() << 24;
    encoderCountLH += Wire.read() << 16;
    encoderCountLH += Wire.read() << 8;
    encoderCountLH += Wire.read();
    rpmLH = Wire.read() << 8;
    rpmLH += Wire.read();
    encoderCountRH = Wire.read() << 24;
    encoderCountRH += Wire.read() << 16;
    encoderCountRH += Wire.read() << 8;
    encoderCountRH += Wire.read();
    rpmRH = Wire.read() << 8;
    rpmRH += Wire.read();

    Serial.print("LH: Cnt: ");
    Serial.print(encoderCountLH);
    Serial.print(" RPM: ");
    Serial.print(rpmLH);
    Serial.print("RH: Cnt: ");
    Serial.print(encoderCountRH);
    Serial.print(" RPM: ");
    Serial.println(rpmRH);

    timestamp = millis();
  }
}
