// Code de Étienne Beaulieu et Emile Bureau
// beae0601 et bure1301
#include "Wire.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <string>

// Définition des pin et adresses des capteurs
#define BARO_ADDRESS 0x77
#define LIGHT_PIN 34
#define HUMIDITY_PIN 16
#define PRECIPITATION_PIN 23
#define WIND_SPEED_PIN 27
#define WIND_DIRECTION_PIN 35

// Définition des UUID nécessaire au Bluetooth
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331911a"  // TODO: Regénérer les UUID spécifiquement pour nous
#define CHARACTERISTIC_TX_UUID "beb5483e-36e1-4688-b7f5-ea07361b261a"

static int32_t twosComplement(int32_t value, uint8_t size);
static void writeByte(int deviceAddr, int dataAddr, int data);
static uint8_t readByte(int deviceAddr, int dataAddr);
static void readBytes(int deviceAddr, int dataAddr, int length, uint8_t* oResult);
static void printArray(int* array, int length);
static void calculateBaro(uint8_t* temperature, uint8_t* press);
static void configureBaro();
static void readBarometer();
static void readLight();
static void readHumitidy();
static void readPrecipitation();
static void readWindDirection();
static void readWindSpeed();
static void configureBluetooth();
static void sendToBluetooth(std::string data);
static void sendToUART(std::string data);
static std::string generateFormatedData();

// Variable for bluetooth
static BLECharacteristic *pCharacteristicTX;

// Variables for barometer
static int16_t c0;
static int16_t c1;
static int32_t c00;
static int32_t c10;
static int16_t c01;
static int16_t c11;
static int16_t c20;
static int16_t c21;
static int16_t c30;
static float tempScaleFactor;
static float pressureScaleFactor;

// Variables for wind sensor
static int windDirectionVoltage[] = {3035, 3519, 3885, 2387, 990, 577, 210, 1700};
static char* windDirectionValues[] = {"N", "NO", "O", "SO", "S", "SE", "E", "NE"};
static unsigned long lastResetTime = 0;
static int windSpeedClick = 0;

// Sensor measurements
static float temperatureBaro = 0.0;
static float pressure = 0.0;
static float light = 0.0;
static float humidity = 0.0;
static float temperatureHumid = 0.0;
static float precipitation = 0.0;
static float windSpeed = 0.0;
static char* windDirection = "";

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Wire.begin();
  configureBluetooth();
  configureBaro();
  
  // Setup des interrupt
  pinMode(PRECIPITATION_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PRECIPITATION_PIN), readPrecipitation, RISING);

  pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), readWindSpeed, RISING);
}

void loop() {
  // Lecture de tous les capteurs
  readBarometer();
  readHumidity();
  readLight();
  readWindDirection();
  
  // Générer le messsage à envoyer en Bluetooth ou en UART
  std::string formatedSensorData = generateFormatedData();
  sendToBluetooth(formatedSensorData);
  
  //sendToUART(formatedSensorData);

  delay(1000);
}

static void configureBluetooth()
{
  // Code pris de l'exemple BLE_Server
  Serial.println("Starting BLE work!");

  BLEDevice::init("Station Meteo beae0601");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristicTX = pService->createCharacteristic(
                                         CHARACTERISTIC_TX_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
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

static void sendToUART(std::string data)
{
  Serial2.println(data.c_str());

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

// For debug purposes
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
  light = analogRead(LIGHT_PIN);
}

static void readBarometer()
{
  uint8_t temperatureBuf[3];
  readBytesI2C(BARO_ADDRESS, 0x03, 3, temperatureBuf);
  uint8_t pressureBuf[3];
  readBytesI2C(BARO_ADDRESS, 0x0, 3, pressureBuf);
  calculateBaro(temperatureBuf, pressureBuf);
}

static void calculateBaro(uint8_t* temperature, uint8_t* press)
{
  // Convert temperature to int
  int32_t tempRaw = twosComplement(((uint32_t)(temperature[0]) << 16) | ((uint32_t)(temperature[1]) << 8) | temperature[2], 24);
  float tempRawScaled = tempRaw/tempScaleFactor;
  
  temperatureBaro = (c0*0.5f) + (c1*tempRawScaled);

  int32_t pressureRaw = twosComplement(((uint32_t)(press[0]) << 16) | ((uint32_t)(press[1]) << 8) | press[2], 24);
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

// Code venant du labo
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
  precipitation += 0.2794/2.0;
}

static void readWindDirection()
{
  // Match la lecture du sensor avec la table de référence pour trouver la direction
  int windDirectionRaw = analogRead(WIND_DIRECTION_PIN);
  for (int i = 0; i < 8; i++)
  {
    if ((windDirectionRaw >= (windDirectionVoltage[i] -100)) && (windDirectionRaw <= (windDirectionVoltage[i] + 100)))
    {
      windDirection = windDirectionValues[i];
      break;
    }
  }
}

static void readWindSpeed()
{
  if (millis() >= lastResetTime + 1000)
  {
    windSpeed = windSpeedClick * 2.4;
    windSpeedClick = 0;
    lastResetTime = millis();
  }
  else
    windSpeedClick += 1;
}

static std::string generateFormatedData()
{
  char data[1024];

  sprintf(data, "\n*************************\n"
                "Pression: %.2f Pa\n"
                "Temperature: %.2f degreC\n"
                "Lumière: %.2f Lx\n"
                "Humidité: %.2f \%\n"
                "Précipitation: %.2f mm\n"
                "Vitesse du vent: %.2f km/h\n"
                "Direction du vent: %s\n",
                pressure, temperatureBaro, light/10.0, humidity, precipitation, windSpeed, windDirection);
  printf(data);
  return (std::string)data;
}