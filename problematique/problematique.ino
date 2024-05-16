#include "Wire.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <string>

#define BARO_ADDRESS 0x77
#define LIGHT_PIN 34
#define HUMIDITY_PIN 16
#define PRECIPITATION_PIN 23
#define WIND_SPEED_PIN 27
#define WIND_DIRECTION_PIN 35

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  // TODO: Regénérer les UUID spécifiquement pour nous
#define CHARACTERISTIC_TX_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

static int32_t twosComplement(int32_t value, uint8_t size);
static void writeByte(int deviceAddr, int dataAddr, int data);
static uint8_t readByte(int deviceAddr, int dataAddr);
static void readBytes(int deviceAddr, int dataAddr, int length, uint8_t* oResult);
static void printArray(int* array, int length);
static void calculateBaro(uint8_t* temperature, uint8_t* pressure);
static void configureBaro();
static void readBarometer();
static void readLight();
static void readHumitidy();
static void readPrecipitation();
static void readWind();
static void configureBluetooth();
static void sendToBluetooth(std::string data);
static std::string generateFormatedData();

// Variable for bluetooth
BLECharacteristic *pCharacteristicTX

// Variable for barometer
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

// Sensor measurements
float temperatureBaro;
float pressure;
float light;
float humidity;
float temperatureHumid;
float precipitation;
float windSpeed;
char* windDirection;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  configureBluetooth();
  configureBaro();
}

void loop() {
  readBarometer();
  readHumidity();
  

  
  std::string formatedSensorData = generateFormatedData();
  sendToBluetooth(formatedSensorData);

  delay(1000);
}

static void configureBluetooth()
{
  Serial.println("Starting BLE work!");

  BLEDevice::init("Station Meteo beae0601");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristicTX = pService->createCharacteristic(
                                         CHARACTERISTIC_TX_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristicTX->setValue("Hello World says Dame Nature");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

static void sendToBluetooth(std::string data)
{
  pCharacteristicTX->setValue(data);
  pCharacteristicTX->notify();

}

static int32_t twosComplement(int32_t value, uint8_t size)
{
  if(value & (1<<(size-1)))
    value -= (1<<size);

  return value;
}

static void writeByteI2C(int deviceAddr, int dataAddr, int data)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(dataAddr);
  Wire.write(data);
  Wire.endTransmission(true);
}


static uint8_t readByteI2C(int deviceAddr, int dataAddr)
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

static void readBytesI2C(int deviceAddr, int dataAddr, int length, uint8_t* oResult)
{
  for (int i = 0; i < length; i++)
  {
    oResult[i] = readByteI2C(deviceAddr, dataAddr + i);
  }
}

static void printArray(int* array, int length)
{
  Serial.println();
  for (int i = 0; i < length; i++)
  {
    Serial.print(array[i]);
    Serial.print(", ");
  }
}

static void readLight()
{
  // TODO
  light = 1.2f;
}

static void readBarometer()
{
  uint8_t temperatureBuf[3];
  readBytesI2C(BARO_ADDRESS, 0x03, 3, temperatureBuf);
  uint8_t pressureBuf[3];
  readBytesI2C(BARO_ADDRESS, 0x0, 3, pressureBuf);
  calculateBaro(temperatureBuf, pressureBuf);
}

static void calculateBaro(uint8_t* temperature, uint8_t* pressure)
{
  // Convert temperature to int
  int32_t tempRaw = twosComplement(((uint32_t)(temperature[0]) << 16) | ((uint32_t)(temperature[1]) << 8) | temperature[2], 24);
  float tempRawScaled = tempRaw/tempScaleFactor;
  
  temperatureBaro = (c0*0.5f) + (c1*tempRawScaled);

  int32_t pressureRaw = twosComplement(((uint32_t)(pressure[0]) << 16) | ((uint32_t)(pressure[1]) << 8) | pressure[2], 24);
  float pressureRawScaled = pressureRaw/pressureScaleFactor;

  pressure = c00 + pressureRawScaled * (c10 + pressureRawScaled * (c20 + pressureRawScaled * c30)) + tempRawScaled * c01 + tempRawScaled * pressureRawScaled * (c11 + pressureRawScaled * c21);

  //Serial.printf("Température: %f\nPression: %f\n", temperatureBaro, pressure);
}

static void configureBaro()
{
  uint8_t coef[18];
  readBytesI2C(BARO_ADDRESS, 0x10, 18, coef);

  // Write pressure config
  int pressureConfig = 0b00100000;
  writeByteI2C(BARO_ADDRESS, 0x06, pressureConfig);
  pressureScaleFactor = 524288.f;

  // Write temperature config
  int temperatureConfig = 0b10100000;
  writeByteI2C(BARO_ADDRESS, 0x07, temperatureConfig);
  tempScaleFactor = 524288.f;

  // Write config for continuous pressure and temperature measurement
  int measurementConfig = 0b00000111;
  writeByteI2C(BARO_ADDRESS, 0x08, measurementConfig);


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

static void readHumidity()
{
  int i, j;
  int duree[42];
  unsigned long pulse;
  byte data[5];
  
  pinMode(HUMIDITY_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(HUMIDITY_PIN, HIGH);
  delay(250);
  digitalWrite(HUMIDITY_PIN, LOW);
  delay(20);
  digitalWrite(HUMIDITY_PIN, HIGH);
  delayMicroseconds(40);
  pinMode(HUMIDITY_PIN, INPUT_PULLUP);
  
  while (digitalRead(HUMIDITY_PIN) == HIGH);
  i = 0;

  do {
        pulse = pulseIn(HUMIDITY_PIN, HIGH);
        duree[i] = pulse;
        i++;
  } while (pulse != 0);
 
  if (i != 42) 
    Serial.printf(" Erreur timing \n"); 

  for (i=0; i<5; i++) {
    data[i] = 0;
    for (j = ((8*i)+1); j < ((8*i)+9); j++) {
      data[i] = data[i] * 2;
      if (duree[j] > 50) {
        data[i] = data[i] + 1;
      }
    }
  }

  if ( (data[0] + data[1] + data[2] + data[3]) != data[4] ) 
    Serial.println(" Erreur checksum");

  humidity = data[0] + (data[1] / 256.0);
  temperatureHumid = data [2] + (data[3] / 256.0);

  //Serial.printf(" Humidite = %4.0f \%%  Temperature = %4.2f degreC \n", humidity, temperatureHumid);
}

static void readPrecipitation()
{
  // TODO
  precipitation = 1.2f;
}

static void readWind()
{
  // TODO
  windDirection = 'E';
  windSpeed = 1.2f;
}

static std::string generateFormatedData()
{
  const char* data;

  sprintf(data, "\n*************************\n"
                "Pression: %f Pa\n"
                "Temperature: %f degreC\n"
                "Lumière: %f \n"
                "Humidité: %f %%\n"
                "Précipitation: %f mm\n"
                "Vitesse du vent: %f m/s\n"
                "Direction du vent: %s\n",
                pressure, temperatureBaro, light, humidity, precipitation, windSpeed, windDirection);
  return (std::string)data;
}