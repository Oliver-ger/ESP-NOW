/*
 ESP-NOW based sensor 
 Sends readings to an ESP-Now server with a fixed mac address
*/
//**************************** Bibliotheken ******************************************/
#include "DHT.h"          // library for DHT22
#include <ESP8266WiFi.h>  // ESP8266 library
#include <Adafruit_Sensor.h>  // Adafruit library
#include <Adafruit_BME280.h>  // Adafruit library for BME280
#include <Wire.h>  
extern "C" {
#include <espnow.h>
}

//************************ locations **********************************************/

#define RAUM "Labor"

//************************* WiFi Access Point *********************************/
// this is the MAC Address of the remote ESP server which receives these sensor readings
// uint8_t remoteMac[] = {0x80, 0x7D, 0x3A, 0x3C, 0x1E, 0x9E};
uint8_t remoteMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

//************************* DHT 22 - sensor ******************************************/

#define DHTPIN 4          // Sensor is connected at GPIO4 ( corresponds to input D2 )   
#define DHTTYPE DHT22     // type is DHT22 sensor
DHT dht(DHTPIN, DHTTYPE); // the sensor DHT22 is now adressd with „dht“

//************************* BME 280 - sensor ****************************************/

Adafruit_BME280 bme;      // I2C he sensor BME280 is now adressd with „bme“

//************************ Sensor - selection ****************************************/
// choose your built-in sensor type
#define BME_SENSOR (false)
#define DHT_SENSOR (true)

//**********************************************************************************/
#define WIFI_CHANNEL 1

//***********************************************************************************/
#define MAX_LOOP_TIME_MS     10000  // max. time Watchdog-Timer, if WiFi or sensor failed

//************************************ send data structure *********************************/
// keep in sync with slave struct
struct __attribute__((packed)) SENSOR_DATA {
  char location[16];
  double temp;
  double humidity;
  double pressure;
  double voltage;
} sensorData;

//*********************************** variables **************************************/ 
int sleepTimeS = 360;  // sleepTimeS in sec.
bool bmeStatus;
int valA0 = 0;  //raw value analog input A0
double temp;
double humidity;
double pressure;
double voltage;
char valTemperature[5];
char valHumidity[5];
char valPressure[7];
char valVoltage[5];
unsigned long startTime;  // start time 
unsigned long monitTime;  // monitoring time
unsigned long stopTime;   // stop time
volatile boolean callbackCalled; // callback has happened



// setup is called once ---------------------------------------------------------/
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println();

  // Start Time 
  Serial.println("Start");
  startTime = millis();  // capture start time
  monitTime = startTime;  // set monitoring time

  // Wait for serial to initialize.
  while(!Serial) { }
  
  // DHT Sensor
  if DHT_SENSOR {
    dht.begin(); // start DHT22 sensor
  }

  // BME 280 Sensor 
  if BME_SENSOR {
    Wire.begin(D2, D1);  // Wire.begin( SDA, SCL )
    Wire.setClock(100000); //This function modifies the clock frequency for I2C communication, 100kHz is standard
    Serial.println(F("BME280 test"));  
    bmeStatus = bme.begin(0x76);  
    if (!bmeStatus) {  
      Serial.println("Could not find a valid BME280 sensor, check wiring!");  
      while (1);  
    }  
    Serial.println("-- Default Test BME--");  
    setup_BMESensor();
  }

  // initialization ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("*** ESP_Now init failed");
    gotoSleepInstant();
  }
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // Register peer
  esp_now_add_peer(remoteMac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);
  // Callback Send function
  esp_now_register_send_cb(OnDataSent);
  callbackCalled = false;
  
}

// Callback when data is sent --------------------------------------------
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  delay(10);
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
  // Goto zu sleep function
  gotoSleepInstant();
}


// function gotoSleepInstant()------------------------------------------/
void gotoSleepInstant(){
  unsigned long totalTime = millis() - startTime;
  //Serial.println();
  //Serial.printf("Total time: %d ms\n", totalTime);
  //Serial.println();
  //Serial.printf("Going to sleep instant for %i secs...\n", sleepTimeS); 
  ESP.deepSleepInstant(sleepTimeS * 1000000, RF_NO_CAL);
}

// funktion "readVoltage" ------------------------------------------------------------------------------
void readVoltage(){
  valA0 = map(analogRead(A0),0,1023,0,55500); // read and scale analog value
  voltage = valA0;
  voltage = voltage / 10000;
  // if battery voltage is less then 3.3V gefallen ist, then increase deepsleep to 30 min.
  if (voltage < 3.3){
    sleepTimeS = 1800;
  }
}


  // Funktion "readDHTSensor"-----------------------------------------------------------------------------
void readDHTSensor(){
    delay(200);
    temp = dht.readTemperature();
    humidity = dht.readHumidity();
    while (!(isValidTemp(temp) && isValidHumidity(humidity))) {
      delay(100);
      Serial.print("d"); // ... print "d" while waiting for valid value
      temp = dht.readTemperature();
      humidity = dht.readHumidity();
      if ((millis() - monitTime) > MAX_LOOP_TIME_MS){
        gotoSleepInstant();  }
    }
}

// Funktion "setupBMESensor"----------------------------------------------------------------------------
void setup_BMESensor(){
    // weather monitoring
    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
}
// Funktion "readBMESensor"-----------------------------------------------------------------------------
void readBMESensor(){
    delay(20);
    temp = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = (bme.readPressure()/100.0F);
    while (!(isValidTemp(temp) && isValidHumidity(humidity) && isValidPressure(pressure))) {
      delay(100);
      Serial.print("b"); // ... print "b" while waiting for valid value
      temp = bme.readTemperature();
      humidity = bme.readHumidity();
      pressure = (bme.readPressure()/100.0F);
      if ((millis() - monitTime) > MAX_LOOP_TIME_MS){
        gotoSleepInstant();  }
    } 
  }

// calculate air pressure to sea level height --------------------------------------------------
double sealevelPressure(double actualPressure) {
  // Exponent = (-1) * 0.44km(Höhe) / 7.8 km
  double exponent = (-1)*(0.44/7.8); 
  double calcPressure = actualPressure / exp(exponent);
  return calcPressure;
}
// function: check if value humidity is valid--------------------------------------------------------------------- 
bool isValidHumidity(const double humidity) {
  return (!isnan(humidity) && humidity >= 0 && humidity <= 100);
}


// function: check if value temperature is valid--------------------------------------------------------------------- 
bool isValidTemp(const double temp) {
  return (!isnan(temp) && temp >= -100 && temp <= 212);
}

// function: check if value pressure is valid--------------------------------------------------------------------- 
bool isValidPressure(const double pressure) {
  return (!isnan(pressure) && pressure >= -100 && pressure <= 1200);
}

// continuously called program ---------------------------------------------------------------/ 
void loop() {
  // connection time until now
  stopTime = millis();  // capture stop time  
  Serial.printf("connection time until now = %d ms\n", stopTime-startTime); 

  // read temperature and humidity from DHT
  if DHT_SENSOR {
    readDHTSensor();
    pressure = 1000.0; // DHT has no pressure, this is a substitute value
    } 

  // read temperature and humidity from BME
  if BME_SENSOR {
    bme.takeForcedMeasurement(); // has no effect in normal mode but in Weather monitoring
    readBMESensor();
  }

  // read Voltage and increase deep sleep time if battery voltage is too low
  readVoltage();
  
  monitTime = millis(); // reset monitoring timer
  unsigned long sensorTime = millis() - startTime;
  Serial.println();
  Serial.printf("Sensor read took %d ms\n", sensorTime);
  
  
  // Set values to send
  strcpy(sensorData.location, RAUM);
  sensorData.temp = temp;
  sensorData.humidity = humidity;
  sensorData.pressure = pressure;
  sensorData.voltage = voltage;

  // Send message via ESP-NOW
  uint8_t bs[sizeof(sensorData)];
  memcpy(bs, &sensorData, sizeof(sensorData));
  esp_now_send(NULL, bs, sizeof(sensorData)); // NULL means send to all peers

  // Ausgabe im seriellen Monitor
  Serial.print("Spannung: ");
  Serial.print(voltage);
  Serial.println(" Volt");
  Serial.print("Temperatur: ");
  Serial.print(temp);
  Serial.println(" Grad Celsius");
  Serial.print("Luftfeuchtigkeit: ");
  Serial.print(humidity); 
  Serial.println(" Prozent");
  if BME_SENSOR {
    Serial.print("Luftdruck: ");
    Serial.print(pressure);
    Serial.println(" mbar"); 
  }
  

  if DHT_SENSOR {
  pinMode(DHTPIN, OUTPUT);    // set DHTPIN an an output
  digitalWrite(DHTPIN, LOW);  // set DHTPIN to LOW before DeepSleep
  }

  Serial.println("Fertig !");
  delay(1000); // time to send data !
}
