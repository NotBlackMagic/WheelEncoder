#include "Wire.h"

#define DEVICE_I2C_ADDR   0x02

#define DEVICE_ID         'N'

enum Register {
  EncoderID = 0x00,
  EncoderData = 0x01,
  EncoderPPR = 0x02
};

//Interface settings
const uint32_t baudrate = 115200;

const uint8_t LEFT_WHEEL = 0;
const uint8_t RIGHT_WHEEL = 1;

//Pin mapping
const uint8_t hallSensor[2] = {A0, A1};

//Encoder processing variables
const int16_t hallMeanValue[2] = {470, 523};
const int16_t hallRisingThreshold[2] = {150, 150};
const int16_t hallFallingThreshold[2] = {100, 100};  
int16_t hallAnalogValue[2] = {0, 0};
uint8_t hallTriggered[2] = {0, 0};

const uint16_t encoderPulsesPerRevolution = 1;
int32_t lastEncoderCount[2] = {0, 0};
int32_t encoderCount[2] = {0, 0};
int16_t rpm[2] = {0, 0};

int16_t rpmCounter = 0;
uint32_t timestamp = 0;

//I2C interface handling variables
Register i2cRegisterRequest = 0;

void setup() {
  //Config GPIOs
  //pinMode(hallSensorI, INPUT_PULLUP);
  //pinMode(hallSensorQ, INPUT_PULLUP);

  //Config Interrupts
  //attachInterrupt(digitalPinToInterrupt(hallSensorI), HallSensorI_IRQ, RISING);
  //attachInterrupt(digitalPinToInterrupt(hallSensorQ), HallSensorQ_IRQ, RISING);

  //Config digital interfaces
  Serial.begin(baudrate);
  Wire.begin(DEVICE_I2C_ADDR);
  Wire.onReceive(I2CRX_IRQ);
  Wire.onRequest(I2CReq_IRQ);
}

void loop() {
  //Read analog values
  hallAnalogValue[LEFT_WHEEL] = analogRead(hallSensor[LEFT_WHEEL]) - hallMeanValue[LEFT_WHEEL];
  hallAnalogValue[RIGHT_WHEEL] = analogRead(hallSensor[RIGHT_WHEEL]) - hallMeanValue[RIGHT_WHEEL];

  //Convert to absolute value
  if(hallAnalogValue[LEFT_WHEEL] < 0) {
    hallAnalogValue[LEFT_WHEEL] = -hallAnalogValue[LEFT_WHEEL];
  }
  if(hallAnalogValue[RIGHT_WHEEL] < 0) {
    hallAnalogValue[RIGHT_WHEEL] = -hallAnalogValue[RIGHT_WHEEL];
  }

  //Check for HAL sensor triggers
  if(hallTriggered[LEFT_WHEEL] == 0 && hallAnalogValue[LEFT_WHEEL] >= hallRisingThreshold[LEFT_WHEEL]) {
    encoderCount[LEFT_WHEEL] += 1;

    hallTriggered[LEFT_WHEEL] = 1;
  }
  else if(hallTriggered[LEFT_WHEEL] == 1 && hallAnalogValue[LEFT_WHEEL] <= hallFallingThreshold[LEFT_WHEEL]) {
    hallTriggered[LEFT_WHEEL] = 0;
  }

  if(hallTriggered[RIGHT_WHEEL] == 0 && hallAnalogValue[RIGHT_WHEEL] >= hallRisingThreshold[RIGHT_WHEEL]) {
    encoderCount[RIGHT_WHEEL] += 1;

    hallTriggered[RIGHT_WHEEL] = 1;
  }
  else if(hallTriggered[RIGHT_WHEEL] == 1 && hallAnalogValue[RIGHT_WHEEL] <= hallFallingThreshold[RIGHT_WHEEL]) {
    hallTriggered[RIGHT_WHEEL] = 0;
  }

  // Serial.print("L:");
  // Serial.print(hallAnalogValue[LEFT_WHEEL]);
  // Serial.print(",R:");
  // Serial.println(hallAnalogValue[RIGHT_WHEEL]);

  if((timestamp + 100) < millis()) {
    if(rpmCounter == 0) {
      //Calculate RPM value, updated every 1s aka 1 Hz
      rpm[LEFT_WHEEL] = ((encoderCount[LEFT_WHEEL] - lastEncoderCount[LEFT_WHEEL]) * 60) / encoderPulsesPerRevolution;
      lastEncoderCount[LEFT_WHEEL] = encoderCount[LEFT_WHEEL];

      rpm[RIGHT_WHEEL] = ((encoderCount[RIGHT_WHEEL] - lastEncoderCount[RIGHT_WHEEL]) * 60) / encoderPulsesPerRevolution;
      lastEncoderCount[RIGHT_WHEEL] = encoderCount[RIGHT_WHEEL];

      rpmCounter = 10;
    }
    rpmCounter -= 1;

    // Serial.print("Encoder: L: ");
    // Serial.print(encoderCount[LEFT_WHEEL]);
    // Serial.print("/");
    // Serial.print(rpm[LEFT_WHEEL]);
    // Serial.print(" R: ");
    // Serial.print(encoderCount[RIGHT_WHEEL]);
    // Serial.print("/");
    // Serial.println(rpm[RIGHT_WHEEL]);

    timestamp = millis();
  }
}

void I2CRX_IRQ(int count) {
  //I2C RX IRQ Handler
  i2cRegisterRequest = Wire.read();
  if(count > 1) {
    //This is a I2C write request, get all bytes with Wire.read()
  }
  //Just in case, clean up I2C rx buffer
  while(Wire.available() == true) {
    Wire.read();
  }
}

void I2CReq_IRQ() {
  uint8_t txDataIndex = 0;
  uint8_t txData[20];
  //I2C data request (read) IRQ Handler
  switch(i2cRegisterRequest) {
    case EncoderID: {
      txData[txDataIndex++] = DEVICE_ID;
      Wire.write(txData, txDataIndex);
      break;
    }
    case EncoderData: {
      txData[txDataIndex++] = encoderCount[LEFT_WHEEL] >> 24;
      txData[txDataIndex++] = encoderCount[LEFT_WHEEL] >> 16;
      txData[txDataIndex++] = encoderCount[LEFT_WHEEL] >> 8;
      txData[txDataIndex++] = encoderCount[LEFT_WHEEL];
      txData[txDataIndex++] = rpm[LEFT_WHEEL] >> 8;
      txData[txDataIndex++] = rpm[LEFT_WHEEL];
      txData[txDataIndex++] = encoderCount[RIGHT_WHEEL] >> 24;
      txData[txDataIndex++] = encoderCount[RIGHT_WHEEL] >> 16;
      txData[txDataIndex++] = encoderCount[RIGHT_WHEEL] >> 8;
      txData[txDataIndex++] = encoderCount[RIGHT_WHEEL];
      txData[txDataIndex++] = rpm[RIGHT_WHEEL] >> 8;
      txData[txDataIndex++] = rpm[RIGHT_WHEEL];
      Wire.write(txData, txDataIndex);
      break;
    }
    case EncoderPPR: {
      txData[txDataIndex++] = encoderPulsesPerRevolution >> 8;
      txData[txDataIndex++] = encoderPulsesPerRevolution;
      Wire.write(txData, txDataIndex);
      break;
    }
  }
}

// void HallSensorI_IRQ() {
//   encoderCount += 1;
// }

// void HallSensorQ_IRQ() {
//   encoderCount += 1;
// }