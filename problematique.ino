// Philippe Mabilleau ing.
// mai 2021
#include "Wire.h"

#define BARO_ADDRESS 0x77


int16_t c0;
int16_t c1;
int32_t c00;
int32_t c10;
int16_t c01;
int16_t c11;
int16_t c20;
int16_t c21;
int16_t c30;

float tempScaleFactor;
float pressureScaleFactor;



int32_t twosComplement(int32_t value, uint8_t size)
{
  if(value & (1<<(size-1)))
    value -= (1<<size);

  return value;
}

void writeByte(int deviceAddr, int dataAddr, int data)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(dataAddr);
  Wire.write(data);
  Wire.endTransmission(true);
}


uint8_t readByte(int deviceAddr, int dataAddr)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(dataAddr);
  Wire.endTransmission(true);
  uint8_t bytesReceived = Wire.requestFrom(deviceAddr, 1);
  //Serial.printf("Data received: %u \n", bytesReceived);

  if ((bool) bytesReceived)
  {
    uint8_t result = Wire.read();
    //Serial.printf("Data: %d \n", result);
    return result;
  }

  return -1;
}

void readBytes(int deviceAddr, int dataAddr, int length, uint8_t* oResult)
{
  for (int i = 0; i < length; i++)
  {
    oResult[i] = readByte(deviceAddr, dataAddr + i);
  }
}

void printArray(int* array, int length)
{
  Serial.println();
  for (int i = 0; i < length; i++)
  {
    Serial.print(array[i]);
    Serial.print(", ");
  }
}

void calculateBaro(uint8_t* temperature, uint8_t* pressure)
{
  // Convert temperature to int
  int32_t tempRaw = twosComplement(((uint32_t)(temperature[0]) << 16) | ((uint32_t)(temperature[1]) << 8) | temperature[2], 24);
  float tempRawScaled = tempRaw/tempScaleFactor;
  
  float adjustedTemperature = (c0*0.5f) + (c1*tempRawScaled);

  int32_t pressureRaw = twosComplement(((uint32_t)(pressure[0]) << 16) | ((uint32_t)(pressure[1]) << 8) | pressure[2], 24);
  float pressureRawScaled = pressureRaw/pressureScaleFactor;

  float adjustedPressure = c00 + pressureRawScaled * (c10 + pressureRawScaled * (c20 + pressureRawScaled * c30)) + tempRawScaled * c01 + tempRawScaled * pressureRawScaled * (c11 + pressureRawScaled * c21);

  Serial.printf("Température: %f\nPression: %f\n", adjustedTemperature, adjustedPressure);
}

static void configureBaro()
{
  uint8_t coef[18];
  readBytes(BARO_ADDRESS, 0x10, 18, coef);

  // Write pressure config
  int pressureConfig = 0b00100000;
  writeByte(BARO_ADDRESS, 0x06, pressureConfig);
  pressureScaleFactor = 524288.f;

  // Write temperature config
  int temperatureConfig = 0b10100000;
  writeByte(BARO_ADDRESS, 0x07, temperatureConfig);
  tempScaleFactor = 524288.f;

  // Write config for continuous pressure and temperature measurement
  int measurementConfig = 0b00000111;
  writeByte(BARO_ADDRESS, 0x08, measurementConfig);


  c0 = twosComplement((coef[0] << 4) | ((coef[1] >> 4) & 0x0F), 12);
  c1 = twosComplement(((coef[1] & 0x0F) << 8) | coef[2], 12);
  c00 = twosComplement((coef[3] << 12) | (coef[4] << 4) | (coef[5] >> 4),20);
  c10 = twosComplement(((coef[5] & 0x0F) << 16) | (coef[6] << 8) | coef[7],20);
  c01 = twosComplement((coef[8] << 8) | coef[9],16);
  c11 = twosComplement((coef[10] << 8) | coef[11],16);
  c20 = twosComplement((coef[12] << 8) | coef[13],16);
  c21 = twosComplement((coef[14] << 8) | coef[15],16);
  c30 = twosComplement((coef[16] << 8) | coef[17],16);
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  configureBaro();
}

void loop() {
  float temperature;
  float pressure;

  while(true)
  {
    uint8_t temperatureBuf[3];
    readBytes(BARO_ADDRESS, 0x03, 3, temperatureBuf);

    uint8_t pressureBuf[3];
    readBytes(BARO_ADDRESS, 0x0, 3, pressureBuf);

    calculateBaro(temperatureBuf, pressureBuf);

    delay(1000);
  }
  


  delay(2000);
}