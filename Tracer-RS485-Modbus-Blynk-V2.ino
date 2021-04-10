// CONNECT THE RS485 MODULE.
// MAX485 module <-> ESP8266
//  - DI -> D10 / GPIO1 / TX
//  - RO -> D9 / GPIO3 / RX
//  - DE and RE are interconnected with a jumper and then connected do eighter pin D1 or D2
//  - VCC to +5V / VIN on ESP8266
//  - GNDs wired together
// -------------------------------------
// You do not need to disconnect the RS485 while uploading code.
// After first upload you should be able to upload over WiFi
// Tested on NodeMCU + MAX485 module
// RJ 45 cable: Green -> A, Blue -> B, Brown -> GND module + GND ESP8266
// MAX485: DE + RE interconnected with a jumper and connected to D1 or D2
//
// Developed by @jaminNZx
// With modifications by @tekk

#include <FS.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <ModbusMaster.h>
#include "settings.h"
#include <WiFiManager.h>

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

const int defaultBaudRate = 115200;
int timerTask1, timerTask2, timerTask3;
float battChargeCurrent, battDischargeCurrent, battOverallCurrent, battChargePower;
float bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;
bool rs485DataReceived = true;
bool loadPoweredOn = true;
String chargingStatus;
char* OTApass = "Password1";
char* BlynkToken = "Dd5XVzR5qSLh6qg_9RpyRnaF80MpPKVw";

#define MAX485_DE D1
#define MAX485_RE_NEG D2

ModbusMaster node;
BlynkTimer timer;

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

// ****************************************************************************

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(defaultBaudRate);

  // Modbus slave ID 1
  node.begin(1, Serial);

  // callbacks to toggle DE + RE on MAX485
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  Serial.println("Connecting to Wifi...");

  WiFiManagerParameter custom_OTApass("OTApass", "OTA Update Password", OTApass, 15);
  WiFiManagerParameter custom_BlynkToken("BlynkToken", "Blynk Token", BlynkToken, 40);
  
  WiFiManager wifiManager;
  WiFi.mode(WIFI_STA);
  //wifiManager.resetSettings();
  wifiManager.addParameter(&custom_OTApass);
  wifiManager.addParameter(&custom_BlynkToken);
  wifiManager.autoConnect("SolarMonitor", "12345678");
  
#if defined(USE_LOCAL_SERVER)
  Blynk.config(BlynkToken, SERVER);
#else
  Blynk.config(BlynkToken);
#endif

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  strcpy(OTApass, custom_OTApass.getValue());
  strcpy(BlynkToken, custom_BlynkToken.getValue());
  
  Serial.println("Connected.");
  Serial.print("Connecting to Blynk...");

  while (!Blynk.connect()) {
    Serial.print(".");
    delay(100);
  }

  Serial.println();
  Serial.println("Connected to Blynk.");
  Serial.println("Starting ArduinoOTA...");

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword((const char *)OTApass);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd of update");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  
  ArduinoOTA.begin();

  Serial.print("ArduinoOTA running. ");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting timed actions...");
  
  timer.setInterval(10000L, readAndUpload);

  Serial.println("Setup OK!");
  Serial.println("----------------------------");
  Serial.println();
}

// --------------------------------------------------------------------------------
  
  // upload values
  void uploadToBlynk() {
    Blynk.virtualWrite(vPIN_PV_POWER,                   pvpower);
    Blynk.virtualWrite(vPIN_PV_CURRENT,                 pvcurrent);
    Blynk.virtualWrite(vPIN_PV_VOLTAGE,                 pvvoltage);
    Blynk.virtualWrite(vPIN_LOAD_CURRENT,               lcurrent);
    Blynk.virtualWrite(vPIN_LOAD_POWER,                 lpower);
    Blynk.virtualWrite(vPIN_BATT_TEMP,                  btemp);
    Blynk.virtualWrite(vPIN_BATT_VOLTAGE,               bvoltage);
    Blynk.virtualWrite(vPIN_BATT_REMAIN,                bremaining);
    Blynk.virtualWrite(vPIN_CONTROLLER_TEMP,            ctemp);
    Blynk.virtualWrite(vPIN_BATTERY_CHARGE_CURRENT,     battChargeCurrent);
    Blynk.virtualWrite(vPIN_BATTERY_CHARGE_POWER,       battChargePower);
    Blynk.virtualWrite(vPIN_BATTERY_OVERALL_CURRENT,    battOverallCurrent);
    Blynk.virtualWrite(vPIN_LOAD_ENABLED,               loadPoweredOn);
    Blynk.virtualWrite(vPIN_CHARGING_STATUS,            chargingStatus);
  }

  void readAndUpload(){
    readPVVoltage();
    readPVCurrent();
    readPVPower();
    readBattVoltage();
    readBattChargeCurrent();
    readBattChargePower();
    readLoadCurrent();
    readLoadPower();
    readBattTemp();
    readBattSOC();
    readLoadState();
    readChargingStatus();
    uploadToBlynk();
  }

  void readPVVoltage(){
    result = node.readInputRegisters(0x3100, 1);
   
    if (result == node.ku8MBSuccess) {    
      pvvoltage = node.getResponseBuffer(0x00) / 100.0f;
      Serial.print("PV Voltage: ");
      Serial.println(pvvoltage);
    } else {
      Serial.println("Read register 0x3100 failed!");
    }
    delay(500);
  }

  void readPVCurrent(){
    result = node.readInputRegisters(0x3101, 1);

    if (result == node.ku8MBSuccess) {
      pvcurrent = node.getResponseBuffer(0x00) / 100.0f;
        Serial.print("PV Current: ");
        Serial.println(pvcurrent);
    } else {
      Serial.println("Read register 0x3101 failed!");
    }
    delay(500);
  }

  void readPVPower(){
    result = node.readInputRegisters(0x3102, 2);
  
    if (result == node.ku8MBSuccess) {  
      pvpower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
      Serial.print("PV Power: ");
      Serial.println(pvpower);
    } else {
      Serial.println("Read register 0x3102 0x3103 failed!");
    }
    delay(500);
  }

  void readBattVoltage(){
    result = node.readInputRegisters(0x3104, 1);

    if (result == node.ku8MBSuccess) {
      bvoltage = node.getResponseBuffer(0x00) / 100.0f;
      Serial.print("Battery Voltage: ");
      Serial.println(bvoltage);
    } else {
      Serial.println("Read register 0x3104 failed!");
    }
    delay(500);
  }

  void readBattChargeCurrent(){
    result = node.readInputRegisters(0x3105, 1);

    if (result == node.ku8MBSuccess) {
      battChargeCurrent = node.getResponseBuffer(0x00) / 100.0f;
      Serial.print("Battery Charge Current: ");
      Serial.println(battChargeCurrent);
    } else {
      Serial.println("Read register 0x3105 failed!");
    }
    delay(500);
  }

  void readBattChargePower(){
    result = node.readInputRegisters(0x3106, 2);

    if (result == node.ku8MBSuccess) {
      battChargePower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16)  / 100.0f;
      Serial.print("Battery Charge Power: ");
      Serial.println(battChargePower);
    } else {
      Serial.println("Read register 0x3106 0x3107 failed!");
    }
    delay(500);
  }

  void readLoadCurrent(){
    result = node.readInputRegisters(0x310D, 1);

    if (result == node.ku8MBSuccess) {
      lcurrent = node.getResponseBuffer(0x00) / 100.0f;
      Serial.print("Load Current: ");
      Serial.println(lcurrent);
    } else {
      Serial.println("Read register 0x310D failed!");
    }
    delay(500);
  }

  void readLoadPower(){
    result = node.readInputRegisters(0x310E, 2);

    if (result == node.ku8MBSuccess) {
      lpower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
      Serial.print("Load Power: ");
      Serial.println(lpower);
    } else {
      Serial.println("Read register 0x310E 0x310F failed!");
    }
    delay(500);
  }

  void readBattTemp(){
    result = node.readInputRegisters(0x3110, 1);

    if (result == node.ku8MBSuccess) {
      btemp = node.getResponseBuffer(0x00) / 100.0f;
      Serial.print("Battery Temperature: ");
      Serial.println(btemp);
    } else {
      Serial.println("Read register 0x3110 failed!");
    }
    delay(500);
  }

  

  void readBattSOC(){
    result = node.readInputRegisters(0x311A, 1);
   
    if (result == node.ku8MBSuccess) {    
      bremaining = node.getResponseBuffer(0x00) / 1.0f;
      Serial.print("Battery Remaining %: ");
      Serial.println(bremaining);
    } else {
      Serial.println("Read register 0x311A failed!");
    }
    delay(500);
  }

  void readLoadState(){
    result = node.readCoils(0x0002, 1);

    if (result == node.ku8MBSuccess){
      loadPoweredOn = node.getResponseBuffer(0x00) / 1.0f;
      Serial.print("Load State: ");
      Serial.println(loadPoweredOn);
    } else {
      Serial.println("Read Load State failed!");
    }
    delay(500);
  }

  void readChargingStatus(){
    result = node.readInputRegisters(0x3201,1);
    if (result == node.ku8MBSuccess) {
      uint16_t output;
      output = (node.getResponseBuffer(0x00) >> 2) & 3;
      switch (output){
        case 0b00:
          chargingStatus = "No Charging";
          break;
        case 0b01:
          chargingStatus = "Float";
          break;
        case 0b10:
          chargingStatus = "Boost";
          break;
        case 0b11:
          chargingStatus = "Equalisation";
          break;
      }
      Serial.print("Charging Status: ");
      
    } else {
      Serial.println("Read Charging Status Failed");
    }
    delay(500);
  }
  

  uint8_t setOutputLoadPower(uint8_t state) {
    Serial.print("Writing coil 0x0002 value to: ");
    Serial.println(state);

    delay(10);
    // Set coil at address 0x0002 (Force the load on/off)
    result = node.writeSingleCoil(0x0002, state);

    if (result == node.ku8MBSuccess) {
      node.getResponseBuffer(0x00);
      Serial.println("Success.");
    }

    return result;
  }
 
  // callback to on/off button state changes from the Blynk app
  BLYNK_WRITE(vPIN_LOAD_ENABLED) {
    uint8_t newState = (uint8_t)param.asInt();
    
    Serial.print("Setting load state output coil to value: ");
    Serial.println(newState);

    result = setOutputLoadPower(newState);


  }
  



void loop()
{
  Blynk.run();
  ArduinoOTA.handle();
  timer.run();
}
