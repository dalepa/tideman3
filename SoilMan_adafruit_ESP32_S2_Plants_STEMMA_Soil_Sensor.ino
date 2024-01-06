/*
    Soilman 1.0
    Maker:  Dan Pancamo
    Updates
*/

#include "Adafruit_seesaw.h"

Adafruit_seesaw ss;

#include <esp_sleep.h>

#define DEEP_SLEEP_DURATION_MS  60000  // 1 minutes in milliseconds
#define UPTIME_BEFORE_SLEEP_MS  60000  // 1 minutes in milliseconds
unsigned long startTime = 0;  // Variable to track uptime


String Version = "SoilMan OTA V1.0 AF-ESP32-S2";                       // Version 
String BoardId = "Soilman.100";         //ESP32 - Victron MPPT 100/30 
const uint64_t sleepTime = 120e6; // 5 minutes in microseconds

// ESP DEEP SLEEP
#include <esp_sleep.h>

// MAX17048 battery support
#include "Adafruit_MAX1704X.h"
Adafruit_MAX17048 maxlipo;


#include <WiFi.h>
#include <WiFiUDP.h>
#include <map>
#include <algorithm>

#include <string.h>




String line;
//WIFI

const char* ssid     = "cam24";
const char* password = "olivia15";

//UDP
int port = 8089;
const char *influxDNS = "bi.pancamo.com";
IPAddress influxIP;
WiFiUDP udp;



int sampleCnt = 0;



//OTA
#include <ElegantOTA.h> 
#include <WiFiClient.h>
#include <WebServer.h>
WebServer webserver(80);

//WIFIMAN
uint8_t DisconnectReason=0;
unsigned long wifiUptime = millis();
unsigned long wifiDowntime = millis();




String dot2dash(String string) {
  String new_string = "";
  for (int i = 0; i < string.length(); i++) {
    if (string.charAt(i) == '.') {
      new_string += '-';
    } else {
      new_string += string.charAt(i);
    }
  }
  return new_string;
}

//WIFIMAN

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WiFi connected OTA");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Version: " + Version);

  if (WiFi.hostByName(influxDNS, influxIP)) {
      Serial.print("Influx IP: ");
      Serial.println(influxIP);
    } else {
      Serial.println("DNS lookup failed for " + String(influxDNS));
    }
  
  wifiDowntime=millis();
  
  // reason 0 on 1st status is ok
  line = String(BoardId + ".wifi.disreason value=" + String(DisconnectReason));
  toInflux(line);  
    
  
  //line = String(BoardId + ".wifi.localip." + dot2dash(String(WiFi.localIP()))  + " value=" + String(DisconnectReason));
  //toInflux(line);

}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  DisconnectReason = info.wifi_sta_disconnected.reason;

  wifiDowntime=millis();

  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(ssid, password);
}



void wifiSetup()
{

  // WIFI RECONNECT
  WiFi.disconnect(true);
  wifiUptime=millis();
  wifiDowntime=millis();
  
  delay(2000);

  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);


  WiFi.begin(ssid, password);
    
  Serial.println();
  Serial.println();
  Serial.println("Wait for WiFi... ");
  
}

void logWifiStatus()
{
  line = String(BoardId + ".wifi.rssi value=" + String(WiFi.RSSI()));
  toInflux(line);

  wifiUptime=millis()-wifiDowntime; 
  line = String(BoardId + ".wifi.uptime value=" + String(wifiUptime/1000));
  toInflux(line);

}

//WIFI REONNECT END-------------------------

//OTA 
void handle_OnConnect() 
{
  webserver.send(200, "text/plain", "Hello from " + Version + " " + BoardId);
}

void otaSetup()
{
  webserver.on("/", handle_OnConnect);
  ElegantOTA.begin(&webserver);    // Start ElegantOTA
  webserver.begin();
}




//Normalize Data, remove the bad data with 5 samples
std::map<String, int> data[5];
// Function prototype
int medianData(std::map<String, int> (&data)[5], String code);

int  medianData(std::map<String, int> (&data)[5], String code) {
  // Access the values of the map array
  int numbers[5];

  for (int i = 0; i < 5; i++) {
    numbers[i] = data[i][code];
  }
  std::sort(numbers, numbers + 5);
  return (numbers[2]);
}



int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

float average (float * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}




void toInflux (String line)
{

      Serial.println(line);

      udp.beginPacket(influxIP, port);
      udp.print(line);
      udp.endPacket();

}

void MAX17048setup()
{
  Serial.println(F("\nAdafruit MAX17048 simple demo"));

  if (!maxlipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    // while (1) delay(10);
  }
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(maxlipo.getChipID(), HEX);
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


void setup() 
{

  Serial.begin(115200);

  //WIFI SETUP  
      wifiSetup();
      otaSetup();


  Serial.println("Setting up seesaw");

  if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1) delay(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }

 // Batter Monitor Setup
  Serial.println("Setting up MAX17048setup");
  MAX17048setup();

  //SLEEP
 // startTime = millis();  // Record the start time
  //esp_sleep_enable_timer_wakeup(UPTIME_BEFORE_SLEEP_MS);

  print_wakeup_reason();

  log(60,1800); //log for X seconds, sleep for Y seconds



}

void logBattery()
{      
        //Serial.print(F("Batt Voltage: ")); Serial.print(maxlipo.cellVoltage(), 3); Serial.println(" V");
        //Serial.print(F("Batt Percent: ")); Serial.print(maxlipo.cellPercent(), 1); Serial.println(" %");
        //Serial.println();

        toInflux(BoardId + ".BattVoltage value=" + String(maxlipo.cellVoltage()));
        toInflux(BoardId + ".BattPercent value=" + String(maxlipo.cellPercent()));
        toInflux(BoardId + ".BattChargeRate value=" + String(maxlipo.chargeRate()));


}


void logSoil()
{

  float tempC = (float) ss.getTemp() * 1.8 + 32.0;
  uint16_t capread = ss.touchRead(0);

  //Serial.print("Temperature: "); Serial.print(tempC); Serial.println("*F");
  //Serial.print("Capacitive: "); Serial.println(capread);

  toInflux(BoardId + ".Soil.Temperature value=" + String(tempC));
  toInflux(BoardId + ".soil.Capacitive value=" + String(capread));

}

void log(int seconds, int sleeptime)
{

  sleeptime = sleeptime*1000000;
  
  for (int i = 0; i < seconds; i++) 
  {
  
    logBattery();
    logSoil();
    delay(1000);  // log every second
  }
  esp_sleep_enable_timer_wakeup(sleeptime);  //sleeptime in micro x 1,000,000 seconds

  esp_deep_sleep_start();

}

void loop() {

  //  webserver.handleClient();


  //  logBattery();
  //  logSoil();
  //  delay(5000);

/*
    if (millis() - startTime >= UPTIME_BEFORE_SLEEP_MS) 
    {
      // Uptime reached, sleep for remaining 5 minutes
      esp_sleep_enable_timer_wakeup(5000000);

      esp_light_sleep_start();
      startTime = millis();
    }
    
*/
}



