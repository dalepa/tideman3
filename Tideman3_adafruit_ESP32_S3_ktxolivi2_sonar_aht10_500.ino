/*
    Tdeman 3.0
    Maker:  Dan Pancamo
    Updates
*/


String Version = "Tideman OTA V4.1 AF-ESP32-S3";                       // Version 
String BoardId = "Tideman3.ktxolivi2-600";         //ESP32 - Victron MPPT 100/30 
const uint64_t sleepTime = 120e6; // 5 minutes in microseconds

//DS18B20 Sensor
// #include <OneWire.h>
// #include <DallasTemperature.h>

// OneWire oneWire(A0); // DS18B20 on pin A0
// DallasTemperature sensors(&oneWire);


//AHT10
#include <Adafruit_AHTX0.h>
#include <Wire.h>
Adafruit_AHTX0 aht;


// ESP DEEP SLEEP
#include <esp_sleep.h>


// MAX17048 battery support
#include "Adafruit_MAX1704X.h"
Adafruit_MAX17048 maxlipo;

#include "HardwareSerial.h"
#include <Adafruit_Sensor.h>
#include "DHTesp.h"

#include <WiFi.h>
#include <WiFiUDP.h>
#include <map>
#include <algorithm>




// Sonar
float SENSORHEIGHTINFEET=61/12;  // The hight of the botton of the sensor in feet

bool newData = false; // Whether new data is available from the sensor
uint8_t buffer[4];  // our buffer for storing data
uint8_t idx = 0;  // our idx into the storage buffer
float avg;
float distance;  // The last measured distance
float distlow;  // The lowest measured distance
float disthigh;  // The highest measured distance
float distmean;  // The middle measured distance
int count = 0;
int loopcnt = 0;
float mDistance[1000];


float WaterLevelHigh;
float WaterLevelLow;
float WaterLevelAvg;



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

//SERIAL ON RX2 on ESP
// HardwareSerial serialVE(2); // VE.Direct port is connected to UART1


//OTA
#include <ElegantOTA.h> 
#include <WiFiClient.h>
#include <WebServer.h>
WebServer webserver(80);

//WIFIMAN
uint8_t DisconnectReason=0;
unsigned long wifiUptime = millis();
unsigned long wifiDowntime = millis();


//DSP11
DHTesp dht;
#define DHT11_PIN 27




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


void MAX17048setup()
{
  Serial.println(F("\nAdafruit MAX17048 simple demo"));

  if (!maxlipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(maxlipo.getChipID(), HEX);
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

void tempToInflux()
{

      //DHT11
      float temperature = dht.getTemperature()  * 1.8 + 32 ;
      float humidity = dht.getHumidity();
      //Serial.printf("Temp = %2.2f\n", temperature);   

      line = String(BoardId + ".battery.temperature value=" + String(temperature));
      toInflux(line);
      line = String(BoardId + ".battery.humidity value=" + String(humidity));
      toInflux(line);

}


void distToInflux(float distance)
{

      String line = String("tideman3." + BoardId + ".distance value=" + String(distance));
      toInflux(line);

}



void logDistance()
{
  if (Serial1.available()) {

    uint8_t c = Serial1.read();
    // Serial1.println(c, HEX);

    // See if this is a header byte
    if (idx == 0 && c == 0xFF) {
      buffer[idx++] = c;
    }
    // Two middle bytes can be anything
    else if ((idx == 1) || (idx == 2)) {
      buffer[idx++] = c;
    }
    else if (idx == 3) {
      uint8_t sum = 0;
      sum = buffer[0] + buffer[1] + buffer[2];
      if (sum == c) {
        distance = ((uint16_t)buffer[1] << 8) | buffer[2];
        newData = true;
      }
      idx = 0;
    }
  }
  
  if (newData) {

    //Serial.printf("getSonarDistance NEW DATA\n");

    mDistance[count] = distance;
    count++;
    
    //Serial.print("Distance: ");
    //Serial.print(distance/25.4/12);
    //Serial.println(" ft");
    
    if (count == 1000)
    {
      //distToInflux(distance/25.4/12);

      qsort(mDistance, count, sizeof(mDistance[0]), sort_desc);

      disthigh = mDistance[10]/25.4;
      distlow = mDistance[count-10]/25.4;

      avg = average(mDistance,count)/25.4;
      count = 0;
      
      WaterLevelHigh=SENSORHEIGHTINFEET-((distlow)/12.0);
      WaterLevelLow=SENSORHEIGHTINFEET-((disthigh)/12.0);
      WaterLevelAvg=SENSORHEIGHTINFEET-((avg)/12.0);


      toInflux(BoardId + ".WaterLevelAvg value=" + String(WaterLevelAvg));
      toInflux(BoardId + ".WaterLevelHigh value=" + String(WaterLevelHigh));
      toInflux(BoardId + ".WaterLevelLow value=" + String(WaterLevelLow));
      toInflux(BoardId + ".DistanceAvg value=" + String(avg/12.0));

      // log other data  here
      logBattery();
      logWifiStatus();
      logAHT10();


    }

  }
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

/*
void logTemperature()
{
    // Request the temperature from the DS18B20 sensor
    sensors.requestTemperatures();
    // Read the temperature value and return it

      // Read and print the temperature
    float temperature = sensors.getTempCByIndex(0);
    temperature = temperature * 1.8 + 32;
    //Serial.printf("Temp = %2.2f\n",  temperature);   

    line = String(BoardId + ".DS18B20.temperature value=" + String(temperature));
    toInflux(line);


}

*/

void logTds()
{
  
  int sensorValue = analogRead(A0); // Read analog value from sensor
  float tdsValue = map(sensorValue, 0, 1023, 0, 1000); // Map to TDS value range


  //Serial.printf("sensorValue Value = %d\n",  sensorValue);   
  //Serial.printf("tdsValue Value = %f\n",  tdsValue); 
  
  line = String(BoardId + ".TDS value=" + String(tdsValue));
  toInflux(line);
  line = String(BoardId + ".TDSraw value=" + String(sensorValue));
  toInflux(line);


}

void logAHT10()
{
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  
  

  float temperature = (float) temp.temperature * 1.8 + 32.0;
 
  line = String(BoardId + ".AHT10.temperature value=" + String(temperature));
  toInflux(line);

  line = String(BoardId + ".AHT10.humidity value=" + String(humidity.relative_humidity));
  toInflux(line);


/*
  // Print the temperature and humidity to the serial port
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°F");
  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println("%");
*/

}

void setup() 
{

  Serial.begin(9600);
  Serial1.begin(9600); // Set the baud rate for the VE.Direct port

  //DHT11 SETUP
    //dht.setup(DHT11_PIN, DHTesp::DHT11);
  
  //DS18B20 Sensor
  // sensors.begin();


  //WIFI SETUP  
      wifiSetup();
      otaSetup();



  // Batter Monitor Setup
  MAX17048setup();

  //AHT10 Setup
    Wire.begin();

    // Initialize the AHT10 sensor
    if (!aht.begin()) {
      Serial.println("Could not find AHT10 sensor!");
      while (1) {
        delay(10);
      }
  }



}

void loop() {

  
    webserver.handleClient();
    logWifiStatus();
    
    // logDistance();

    /* debug
    logBattery();
    logWifiStatus();
    logAHT10();
*/




}



