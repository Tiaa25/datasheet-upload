#include <DHT.h>
#include <SPI.h>
#include <SD.h>
#include <MQUnifiedsensor.h>
#include "MICS6814.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <WiFiS3.h>
#include <WiFiHttpClient.h>

/************************ Sensor Configurations *******************************/
#define dhtPin 2
#define dhtType DHT22
DHT dht(dhtPin, dhtType);

#define chipSelect 10
File dataFile;

#define         Board                   ("Arduino UNO")
#define         Pin                     (A4)
#define         Type                    ("MQ-2") //MQ2
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (10) // For arduino UNO/MEGA/NANO
#define         RatioMQ2CleanAir        (9.83)
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

 
#define pinCO A2
#define pinNO2 A0
#define pinNH3 A1
MICS6814 gas(pinCO, pinNO2, pinNH3);

TinyGPSPlus gps;
const int pmPin = A5; // PM sensor pin

/************************ Network Configurations *******************************/
const char* ssid = "Vm";
const char* password = "Pumpkin30";
const char* server = "192.168.45.141"; // Server IP
const char* uploadPath = "/project/tesr.php"; // Path to PHP script
WiFiClient wifiClient;
WiFiHttpClient httpClient = WiFiHttpClient(wifiClient, server, 80);

/************************ Global Variables *******************************/
const int timeZoneOffset = 7; // Adjust for your time zone
bool fileExists = false;

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);

    Serial.println("Initializing WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("WiFi connected.");

    Serial.println("Initializing DHT sensor...");
    dht.begin();

    Serial.println("Initializing SD card...");
    if (!SD.begin(chipSelect)) {
        Serial.println("SD card initialization failed!");
        while (true); // Stop here if SD card fails
    }

    Serial.println("Calibrating MQ2 sensor...");
    MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ2.setA(574.25); MQ2.setB(-2.222);
    MQ2.init();
    float calcR0 = 0;
    for (int i = 1; i <= 10; i++) {
        MQ2.update();
        calcR0 += MQ2.calibrate(9.83);
        Serial.print("Calibration step "); Serial.println(i);
    }
    MQ2.setR0(calcR0 / 10);

    Serial.println("Calibrating MICS6814 sensor...");
    gas.calibrate();
    
    Serial.println("Setup complete.");
}


void loop() {
  // --- SENSOR DATA COLLECTION ---
  float humidity = dht.readHumidity();
  float tempC = dht.readTemperature();
  float tempF = dht.readTemperature(true);

  float coPPM = gas.measure(CO);
  float nh3PPM = gas.measure(NH3);
  float no2PPM = gas.measure(NO2);

  MQ2.update();
  float hidrocarbonPPM = MQ2.readSensor();

  int pmValue = analogRead(pmPin);
  float pmVoltage = pmValue * (5.0 / 1023.0);
  float pmDensity = max((pmVoltage - 0.6) / 0.5, 0.0);

  // --- GPS DATA COLLECTION ---
  float latitude = 0, longitude = 0, speedKmph = 0;
  String date = "Invalid", time = "Invalid";

  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());

    if (gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      speedKmph = gps.speed.kmph();
    }
    if (gps.time.isValid()) {
      int hour = gps.time.hour() + timeZoneOffset;
      if (hour >= 24) hour -= 24;
      time = String(hour) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
    }
    if (gps.date.isValid()) {
      date = String(gps.date.day()) + "/" + String(gps.date.month()) + "/" + String(gps.date.year());
    }
  }

  // --- SAVE TO SD ---
  dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (dataFile) {
    if (!fileExists) {
            dataFile.println("humidity,temp_c,temp_f,co_ppm,nh3_ppm,no2_ppm,hidrocarbon_ppm,latitude,longitude,speed_kmph,pm_density,date,time");
            fileExists = true; // Tandai bahwa file sudah ada
        }
    dataFile.print(humidity); dataFile.print(",");
    dataFile.print(tempC); dataFile.print(",");
    dataFile.print(tempF); dataFile.print(",");
    dataFile.print(coPPM); dataFile.print(",");
    dataFile.print(nh3PPM); dataFile.print(",");
    dataFile.print(no2PPM); dataFile.print(",");
    dataFile.print(hidrocarbonPPM); dataFile.print(",");
    dataFile.print(latitude, 6); dataFile.print(",");
    dataFile.print(longitude, 6); dataFile.print(",");
    dataFile.print(speedKmph); dataFile.print(",");
    dataFile.print(pmDensity); dataFile.print(",");
    dataFile.print(date); dataFile.print(",");
    dataFile.println(time);
    dataFile.close();
  }

  // --- DEBUG OUTPUT ---
  Serial.print("Humidity: "); Serial.println(humidity);
  Serial.print("Temp (C): "); Serial.println(tempC);
  Serial.print("Temp (F): "); Serial.println(tempF);
  Serial.print("CO (PPM): "); Serial.println(coPPM);
  Serial.print("NH3 (PPM): "); Serial.println(nh3PPM);
  Serial.print("NO2 (PPM): "); Serial.println(no2PPM);
  Serial.print("Hidrocarbon (PPM): "); Serial.println(hidrocarbonPPM);
  Serial.print("Latitude: "); Serial.println(latitude, 6);
  Serial.print("Longitude: "); Serial.println(longitude, 6);
  Serial.print("Speed (Kmph): "); Serial.println(speedKmph);
  Serial.print("PM Density: "); Serial.println(pmDensity);
  Serial.print("Date: "); Serial.println(date);
  Serial.print("Time: "); Serial.println(time);

  // --- SEND DATA TO SERVER ---
  sendCSVtoServer();
  delay(2000); // Delay 1 minute
}

void sendCSVtoServer() {
  dataFile = SD.open("datalog.csv", FILE_READ);
  if (!dataFile) {
    Serial.println("Failed to open file for upload");
    return;
  }

  String csvData = "";
  while (dataFile.available()) {
    csvData += char(dataFile.read());
  }
  dataFile.close();

  httpClient.beginRequest();
  httpClient.post(uploadPath);
  httpClient.sendHeader("Content-Type", "text/plain");
  httpClient.sendHeader("Content-Length", csvData.length());
  httpClient.beginBody();
  httpClient.print(csvData);
  httpClient.endRequest();

  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();
  Serial.print("Server Response: "); Serial.println(statusCode);
  Serial.print("Response Body: "); Serial.println(response);
}
