#include <Arduino.h>
#include <Wire.h>
#include <IWatchdog.h>
#include <SensirionI2CScd4x.h>
#include "SN-GCJA5.h"

#define LED_PIN PA5
#define ERROR_NONE  0x00000000lu
#define ERROR_I2C   0xCCCCCCCClu
#define ERROR_INIT  0xFFFFFFFFlu

#define I2C_ADDRESS 15
TwoWire WireSlave;

#define I2C_TX_DATA_SIZE 27
uint16_t sensor_processor_model = 0x0000u;
uint8_t i2c_tx_data[I2C_TX_DATA_SIZE] = { 0 };

uint32_t device_status_error = ERROR_INIT;

// Read Measurement
uint16_t co2;
float temperature;
float humidity;

float latest_co2 = 0.0f;
float latest_temp = 0.0f;
float latest_humid = 0.0f;

float latest_pm1_0 = 0.0f;
float latest_pm2_5 = 0.0f;
float latest_pm10 = 0.0f;

float data_1 = 0.0f;
float data_2 = 0.0f;
float data_3 = 0.0f;

uint16_t error;
char errorMessage[256];

uint32_t float2hex = 0;

bool scd_is_init = false;

SFE_PARTICLE_SENSOR myAirSensor;
SensirionI2CScd4x scd4x;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(PA4, OUTPUT);
  digitalWrite(PA5, HIGH);
  digitalWrite(PA4, HIGH);

  Wire.setSCL(PB6); 
  Wire.setSDA(PB7);
  Wire.begin();     

  UpdateTrinityProtocol();
  WireSlave.setSCL(PA11);
  WireSlave.setSDA(PA12);
  WireSlave.begin(I2C_ADDRESS);
  WireSlave.onRequest(requestEvent);     // register event

  scd4x.begin(Wire); 
  myAirSensor.begin(Wire);



  IWatchdog.begin(5000000);
}

void loop()
{
  delay(1000);
  IWatchdog.reload();
  static uint32_t i_scd = 0;
  static uint32_t i_pana = 0;
  uint8_t address = scan_i2c();
  if (address == 0x62){
    if (!scd_is_init){
      sensor_processor_model = 1;
      error = scd4x.startPeriodicMeasurement();
      if (error) {
        device_status_error = ERROR_INIT;
      }
      scd_is_init = true;
      i_scd = 0;
      i_pana = 0;
    }

    if (++i_scd % 2 == 0) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(PA4, HIGH);
    }
    else {
        digitalWrite(LED_PIN, LOW);
        digitalWrite(PA4, LOW);
    }
    if(i_scd%5==0){
      error = scd4x.readMeasurement(co2, temperature, humidity);
      if (error) {
        device_status_error = ERROR_I2C;
      } 
      else {
        latest_co2 = (float)co2;
        latest_temp = temperature;
        latest_humid = humidity;
        data_1 = latest_co2;
        data_2 = latest_temp;
        data_3 = latest_humid;
        device_status_error = ERROR_NONE;
      }
      UpdateTrinityProtocol();
      }
  }
  else if (address == 0x33){
    sensor_processor_model = 2;
    scd_is_init = false;
    if (++i_pana % 2 == 0) {
      digitalWrite(PA4, HIGH);
    }
    else {
        digitalWrite(PA4, LOW);
    }

    auto pm1_0 = myAirSensor.getPM1_0();
    auto pm2_5 = myAirSensor.getPM2_5();
    auto pm10 = myAirSensor.getPM10();

    if (pm1_0.first == SFE_PARTICLE_SENSOR::SNGCJA5_ERROR_NONE &&
        pm2_5.first == SFE_PARTICLE_SENSOR::SNGCJA5_ERROR_NONE &&
        pm10.first == SFE_PARTICLE_SENSOR::SNGCJA5_ERROR_NONE)
    {
      device_status_error = ERROR_NONE;
    }
    else
    {
      device_status_error = ERROR_I2C;
    }

    latest_pm1_0 = pm1_0.second;
    latest_pm2_5 = pm2_5.second;
    latest_pm10 = pm10.second;
    data_1 = latest_pm1_0;
    data_2 = latest_pm2_5;
    data_3 = latest_pm10;

    UpdateTrinityProtocol();
  }
  else {
    sensor_processor_model = 0;
    i_scd = 0;
    i_pana = 0;
    data_1 = 0;
    data_2 = 0;
    data_3 = 0;
    scd_is_init = false;
    UpdateTrinityProtocol();
  }
  
  IWatchdog.reload();
}

void UpdateTrinityProtocol()
{
    i2c_tx_data[0]  = I2C_TX_DATA_SIZE - 1;
    i2c_tx_data[1]  = (sensor_processor_model >> 8) & 0xFF;
    i2c_tx_data[2]  = (sensor_processor_model >> 0) & 0xFF;

    i2c_tx_data[3]  = 0x00; // Data num 0
    i2c_tx_data[4]  = 0x06; // Unsigned int 32 bits
    i2c_tx_data[5]  = (device_status_error >> 24) & 0xFF;
    i2c_tx_data[6]  = (device_status_error >> 16) & 0xFF;
    i2c_tx_data[7]  = (device_status_error >> 8) & 0xFF;
    i2c_tx_data[8]  = (device_status_error >> 0) & 0xFF;

    memcpy(&float2hex, &data_1, sizeof(float));
    i2c_tx_data[9]  = 0x01; // Data num 1
    i2c_tx_data[10] = 0x09; // Float
    i2c_tx_data[11] = (float2hex >> 24) & 0xFF;
    i2c_tx_data[12] = (float2hex >> 16) & 0xFF;
    i2c_tx_data[13] = (float2hex >> 8) & 0xFF;
    i2c_tx_data[14] = (float2hex >> 0) & 0xFF;

    memcpy(&float2hex, &data_2, sizeof(float));
    i2c_tx_data[15] = 0x02; // Data num 2
    i2c_tx_data[16] = 0x09; // Float
    i2c_tx_data[17] = (float2hex >> 24) & 0xFF;
    i2c_tx_data[18] = (float2hex >> 16) & 0xFF;
    i2c_tx_data[19] = (float2hex >> 8) & 0xFF;
    i2c_tx_data[20] = (float2hex >> 0) & 0xFF;

    memcpy(&float2hex, &data_3, sizeof(float));
    i2c_tx_data[21] = 0x03; // Data num 3
    i2c_tx_data[22] = 0x09; // Float
    i2c_tx_data[23] = (float2hex >> 24) & 0xFF;
    i2c_tx_data[24] = (float2hex >> 16) & 0xFF;
    i2c_tx_data[25] = (float2hex >> 8) & 0xFF;
    i2c_tx_data[26] = (float2hex >> 0) & 0xFF;
}

void requestEvent()
{
  WireSlave.write(i2c_tx_data, I2C_TX_DATA_SIZE);
}

uint8_t scan_i2c()
{
  uint8_t error, address;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      return address;
    }
  }
  return 0xFF;
}
