/***************************************************************************
  BLE AirQuality Beacon for Geotab - ESP32 edition
  Measured values: Temperature, Humidity, PM 2.5, PM 10
  Sources:
  - Adafruit Sensor library: https://github.com/adafruit/Adafruit_Sensor
  - Adafruit BME280 library: https://github.com/adafruit/Adafruit_BME280_Library
  - Alvaro Valdebenito's PMSerial library: https://github.com/avaldebe/PMserial
 ***************************************************************************/

// Libraries to be included
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PMserial.h>  // Arduino library for PM sensors with serial interface
#include "esp_sleep.h"

// Defining both hardware serial ports on the LOLIN32 board for the PM sensor
SerialPM pms1(PMSx003, Serial1);
SerialPM pms2(PMSx003, Serial2);

// BME280 sensor via I2C
Adafruit_BME280 bme;
// Onboard LED
unsigned int LED = 5;
// Battery level used as status indicator. 
int status = 30;

void setup() {
  pinMode(LED, OUTPUT);
  // Debug serial output
  Serial.begin(115200);
  Serial.println("--------------------------------");

  // Initializing the PM sensors
  pms1.init();
  pms2.init();
  
  // Initializing the BME280 sensor
  bool bmestatus;
  bmestatus = bme.begin(0x76);  
  if (!bmestatus) {
      Serial.println("BME280 not available, please check the wiring!");
      status = status + 1;
  }
   
  digitalWrite(LED, LOW);
  
  // Initializing the BLE server
  BLEDevice::init("Geotab AirQBee");
 
  BLEServer *pServer = BLEDevice::createServer();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  BLEAdvertisementData advertisementData;

  // Setting the necessary flags
  advertisementData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

  // Reading temperature
  int tempval = 0;
  tempval = bme.readTemperature();
  Serial.printf("Temperature: %2d °C\n",tempval);  

  // Reading humidity
  int humival = 0;
  humival = bme.readHumidity();
  Serial.printf("Humidity: %2d",humival);
  Serial.print(" %\n");

  // Reading PM1.0, PM 2.5 and PM 10
  int pm25val = 0;
  int pm10val = 0;   
  
  pms1.read();
  if(pms1){  // PMS1 ok
    // print formatted results
    Serial.printf("PMS1: PM2.5 %2d ug/m3, PM10 %2d ug/m3\n",pms1.pm25,pms1.pm10);
    pms2.read();
    if(pms2){  // PMS1 and PMS1 ok
      // print formatted results
      Serial.printf("PMS2: PM2.5 %2d ug/m3, PM10 %2d ug/m3\n",pms2.pm25,pms2.pm10);
      pm25val = (pms1.pm25 + pms2.pm25) / 2;
      pm10val = (pms1.pm10 + pms2.pm10) / 2;      
    }
    else { //PMS1 ok, but PMS2 not 
      status = status + 4;
      Serial.println("PMS2 not available, please check the wiring!");     
      pm25val = pms1.pm25;
      pm10val = pms1.pm10; 
    }
  }
  else { // PMS1 not ok
    status = status + 2;
    Serial.println("PMS1 not available, please check the wiring!");
    pms2.read();
    if(pms2){  // PMS1 not ok, but PMS2 ok
      // print formatted results
      Serial.printf("PMS2: PM2.5 %2d ug/m3, PM10 %2d ug/m3\n",pms2.pm25,pms2.pm10);
      pm25val = pms2.pm25;
      pm10val = pms2.pm10;        
    }
    else { //both PMS1 and PMS2 are not ok
      status = status + 4;
      Serial.println("PMS2 not available, please check the wiring!");
    }
  }
  if (pm25val < 0) pm25val = 0;
  else if (pm25val > 5000) pm25val = 5000;
  if (pm10val < 0) pm10val = 0;
  else if (pm10val > 5000) pm10val = 5000;

  Serial.printf("Average PM2.5: %2d ug/m3",pm25val);
  Serial.print(" %\n");
  Serial.printf("Average PM10:  %2d ug/m3\n",pm10val);

  // Preparing the values for the payload
  float tempfval = tempval;
  byte* temparray = (byte*) &tempfval;
  float humifval = humival;
  byte* humiarray = (byte*) &humifval;
  float pm25fval = pm25val;
  byte* pm25array = (byte*) &pm25fval;
  float pm10fval = pm10val;
  byte* pm10array = (byte*) &pm10fval;
  
  // Building the payload
  char manfdata[21];
  // Geotab's company ID
  manfdata[0]= 0x75;
  manfdata[1]= 0x02;
  // Advertising packet version number
  manfdata[2]= 0x00;
  // TX Power Level
  manfdata[3]= 0xC6;
  // Battery level - used as SW version number and status indicator (v3 + returncode)
  manfdata[4]= char(status);
  // Temperature
  manfdata[5]= 0x07;
  manfdata[6]= 0x00;
  manfdata[7]= temparray[2];
  manfdata[8]= temparray[3]; 
  // Relative humidity
  manfdata[9]= 0x09;
  manfdata[10]= 0x00;
  manfdata[11]= humiarray[2];
  manfdata[12]= humiarray[3];
  // PM 2.5
  manfdata[13]= 0x0D;
  manfdata[14]= 0x00;
  manfdata[15]= pm25array[2];
  manfdata[16]= pm25array[3];
  // PM 10
  manfdata[17]= 0x0E;
  manfdata[18]= 0x00;
  manfdata[19]= pm10array[2];
  manfdata[20]= pm10array[3];

  Serial.print("Payload: ");
  // Debug output
  for (int i = 0; i < sizeof(manfdata); ++i) {
   Serial.print(manfdata [i], HEX);
   Serial.print(" ");
  }

  // Setting the payload for the advertisement data
  advertisementData.setManufacturerData(std::string(manfdata, 25));
  pAdvertising->setAdvertisementData(advertisementData);
  digitalWrite(LED, HIGH);

  // Starting the advertisement
  pAdvertising->start();
  delay(100);
  // Stopping the advertisement
  pAdvertising->stop();
  // Deep sleep until the next cycle
  Serial.println();
  //Serial.println("Entering deep sleep...");
  Serial.println("----------------------------------");
      
  esp_deep_sleep(900000LL);
}

void loop() {
}
