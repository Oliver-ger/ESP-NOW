/*
 ESP-NOW based sensor 
 Sends readings to an ESP-Now server with a fixed mac address
*/
//**************************** Bibliotheken ******************************************/
#include "DHT.h"          // Implementierung der ESP8266 WiFi Bibliothek
#include <ESP8266WiFi.h>  // ESP8266 Bibliothek laden
#include <Adafruit_Sensor.h>  // Adafruit Bibliothek
#include <Adafruit_BME280.h>  // Adafruit Bibliothek für BME280
#include <Wire.h>  
extern "C" {
#include <espnow.h>
}

//************************ Räume **********************************************/

#define RAUM "Labor"

//************************* WiFi Access Point *********************************/
// this is the MAC Address of the remote ESP server which receives these sensor readings
// uint8_t remoteMac[] = {0x80, 0x7D, 0x3A, 0x3C, 0x1E, 0x9E};
uint8_t remoteMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};

//************************* DHT 22 - Sensor ******************************************/

#define DHTPIN 4          // Der Sensor wird an GPIO4 (entspricht dem Eingang x) angeschlossen    
#define DHTTYPE DHT22     // Es handelt sich um den DHT22 Sensor
DHT dht(DHTPIN, DHTTYPE); // Der Sensor wird ab jetzt mit „dht“ angesprochen

//************************* BME 280 - Sensor ****************************************/

Adafruit_BME280 bme;      // I2C Der Sensor wird ab jetzt mit „bme“ angesprochen

//************************ Sensor - Auswahl ****************************************/
#define BME_SENSOR (false)
#define DHT_SENSOR (true)

//**********************************************************************************/
#define WIFI_CHANNEL 1

//***********************************************************************************/
#define MAX_LOOP_TIME_MS     10000  // Watchdog-Timer, wenn WLAN oder DHT ausgefallen ist

//************************************ Send-Data *******************************************/
// keep in sync with slave struct
struct __attribute__((packed)) SENSOR_DATA {
  char Ort[16];
  double temp;
  double humidity;
  double pressure;
  double voltage;
} sensorData;

//*********************************** Variables **************************************/ 
int sleepTimeS = 360;  // sleepTimeS in sec.
bool toggleFlag;   // Wenn Toggle-Flag = 1 ist, dann soll ein Deep Sleep nach Reboot erfolgen
bool bmeStatus;
int valA0 = 0;  //Rohwert Analogeingang A0
double temp;
double humidity;
double pressure;
double voltage;
char valTemperature[5];
char valHumidity[5];
char valPressure[7];
char valVoltage[5];
unsigned long startTime;  // Startzeit
unsigned long monitTime;  // Überwachungszeit
unsigned long stopTime;   // Stopzeit
volatile boolean callbackCalled; // Callback hat stattgefunden



// Setup wird nur einmal durchlaufen ---------------------------------------------------------/
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println();

  // Start Time 
  Serial.println("Start");
  startTime = millis();  // Startzeit erfassen
  monitTime = startTime;  // Überwachungszeit erfassen

  // Wait for serial to initialize.
  while(!Serial) { }
  
  // DHT Sensor
  if DHT_SENSOR {
    dht.begin(); //DHT22 Sensor starten
  }

  // BME 280 Sensor 
  if BME_SENSOR {
    Wire.begin(D2, D1);  // Wire.begin( SDA, SCL )z.B. D3,D4
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

  // Inititialisierung ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("*** ESP_Now init failed");
    gotoSleepInstant();
  }
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // Register peer
  esp_now_add_peer(remoteMac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);
  // Callback Send Funktionsaufruf
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
  // Gehe zu Sleep Funktion
  gotoSleepInstant();
}


// Funktion gotoSleepInstant()------------------------------------------/
void gotoSleepInstant(){
  unsigned long totalTime = millis() - startTime;
  //Serial.println();
  //Serial.printf("Total time: %d ms\n", totalTime);
  //Serial.println();
  //Serial.printf("Going to sleep instant for %i secs...\n", sleepTimeS); 
  ESP.deepSleepInstant(sleepTimeS * 1000000, RF_NO_CAL);
}

// Funktion "readVoltage" ------------------------------------------------------------------------------
void readVoltage(){
  valA0 = map(analogRead(A0),0,1023,0,55500); // Analogwert einlesen und normieren
  voltage = valA0;
  voltage = voltage / 10000;
  // Wenn die Batteriespannung unter 3.3V gefallen ist, dann Deepsleep auf 30min. erhöhen
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
      Serial.print("d"); // ... sollen Punkte ausgegeben werden. Die Punkte dienen als Kontrollelement.
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
      Serial.print("b"); // ... sollen Punkte ausgegeben werden. Die Punkte dienen als Kontrollelement.
      temp = bme.readTemperature();
      humidity = bme.readHumidity();
      pressure = (bme.readPressure()/100.0F);
      if ((millis() - monitTime) > MAX_LOOP_TIME_MS){
        gotoSleepInstant();  }
    } 
  }

// Funktion Luftdruck auf Meereshöhe umrechnen-----------------------------------------------------------
double sealevelPressure(double actualPressure) {
  // Exponent = (-1) * 0.44km(Höhe) / 7.8 km
  double exponent = (-1)*(0.44/7.8); 
  double calcPressure = actualPressure / exp(exponent);
  return calcPressure;
}
// Funktion Wert Feuchtigkeit gültig--------------------------------------------------------------------- 
bool isValidHumidity(const double humidity) {
  return (!isnan(humidity) && humidity >= 0 && humidity <= 100);
}


// Funktion Wert Temperatur gültig--------------------------------------------------------------------- 
bool isValidTemp(const double temp) {
  return (!isnan(temp) && temp >= -100 && temp <= 212);
}

// Funktion Wert Druck gültig--------------------------------------------------------------------- 
bool isValidPressure(const double pressure) {
  return (!isnan(pressure) && pressure >= -100 && pressure <= 1200);
}

// Kontinuierliches Programm ---------------------------------------------------------------/ 
void loop() {
  // Verbindungszeit bisher
  stopTime = millis();  // Stopzeit erfassen  
  Serial.printf("Verbindungszeit bisher = %d ms\n", stopTime-startTime); 

  // read temperature and humidity from DHT
  if DHT_SENSOR {
    readDHTSensor(); } 

  // read temperature and humidity from BME
  if BME_SENSOR {
    bme.takeForcedMeasurement(); // has no effect in normal mode but in Weather monitoring
    readBMESensor();
  }

  // read Voltage und erhöhe Deepsleep-Zeit, wenn Batteriespannung < 3.3V
  readVoltage();
  // voltage = 4.0;
  
  monitTime = millis(); // Überwachungstimer zurücksetzen
  unsigned long sensorTime = millis() - startTime;
  Serial.println();
  Serial.printf("Sensor read took %d ms\n", sensorTime);
  
  
  // Set values to send
  strcpy(sensorData.Ort, RAUM);
  sensorData.temp = temp;
  sensorData.humidity = humidity;
  sensorData.pressure = 1000.0;
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
  pinMode(DHTPIN, OUTPUT);    // DHTPIN als Ausgang deklarieren
  digitalWrite(DHTPIN, LOW);  // und vor dem DeepSleep auf LOW setzen
  }

  Serial.println("Fertig !");
  delay(1000); // Zeit zum Senden geben !
}
