/**
 * ESP-NOW Receiver with own sensor
 * 
 * This program is written for an ESP8266 to receive weather data via ESP-Now and
 * shows this data on an own display.
 * The data from the built-in sensor is shown in the upper half of the TFT display.
 * The received data from another ESP8266 is shown in the lower half of the TFT display.
 * The ESP-Now basic program is extracted from here:
 * https://github.com/HarringayMakerSpace/ESP-Now
 * 
 * You have the possibilty to choose either the DHT22 sensor or the BME280 sensor
 * in the receive station.
 * 
 */
 //**************************** libraries ******************************************/
#include <ESP8266WiFi.h>
#include "DHT.h"              // library for DHT22
#include <Adafruit_Sensor.h>  // Adafruit library
#include <Adafruit_BME280.h>  // Adafruit library for BME280
#include <Wire.h>  
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Fonts/FreeMonoOblique9pt7b.h>
#include <SPI.h>
extern "C" {
  #include <espnow.h>
  #include "user_interface.h"
}
//************************* DHT 22 - sensor ******************************************/

#define DHTPIN 4          // Sensor is connected at GPIO4 ( corresponds to input D2 )   
#define DHTTYPE DHT22     // type is DHT22 sensor
DHT dht(DHTPIN, DHTTYPE); // the sensor DHT22 is now adressd with „dht“

//************************* BME 280 - sensor ****************************************/

Adafruit_BME280 bme;      // I2C the sensor BME280 is now adressd with „bme“

//************************ Sensor - selection ****************************************/
// choose your built-in sensor type
#define BME_SENSOR (false)
#define DHT_SENSOR (true)

//************************ Display *******************************************/

#define TFT_CS        D8 //CS
#define TFT_RST       D6 //Reset
#define TFT_DC        D0 //A0/DC
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
void setRotation(uint8_t rotation);

//************************ variables *******************************************/
int valA0 = 0;  // raw value analog input A0
bool bmeStatus;
double temp;
double humidity;
double pressure;
double voltage;
int timer1;
int timer2;
int minuteCountRaum = 0;
int minuteCountLabor = 0;
String  strLocation ="";
volatile boolean NewDataReceived = false; // volatile variable, because it is influenced
                                          // during an interrupt

//****************************************************************************/
/* Set a private Mac Address
 *  http://serverfault.com/questions/40712/what-range-of-mac-addresses-can-i-safely-use-for-my-virtual-machines
 * Note: the point of setting a specific MAC is so you can replace this Gateway ESP8266 device with a new one
 * and the new gateway will still pick up the remote sensors which are still sending to the old MAC 
 */
uint8_t mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
void initVariant() {
  WiFi.mode(WIFI_AP);
  wifi_set_macaddr(SOFTAP_IF, &mac[0]);
}

//*********************** reveive data structure *************************************/
// keep in sync with slave struct
struct __attribute__((packed)) SENSOR_DATA {
  char location[16];
  double temp;
  double humidity;
  double pressure;
  double voltage;
} sensorData;


//-----------------------------------------------------------------------------/
void initEspNow() {
  if (esp_now_init()!=0) {
    Serial.println("*** ESP_Now init failed");
    ESP.restart();
  }
  // Configuration on ESP_NOW
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Configure callback function
  esp_now_register_recv_cb(OnDataRecv);
    // Investigate: There's little doc on what can be done within this method. If its like an ISR
    // then it should not take too long or do much I/O, but writing to Serial does appear to work ok
}


// Callback function that will be executed when data is received----------------------/
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  // set flag if new data received
  NewDataReceived = true;
}

// print new data on serial interface --------------------------------/
void PrintNewDataSerial(){
  Serial.print("Ort: ");
  Serial.println(sensorData.location);
  Serial.print("Temperatur: ");
  Serial.println(sensorData.temp);
  Serial.print("Luftfeuchtigkeit: ");
  Serial.println(sensorData.humidity);
  Serial.print("Luftdruck: ");
  Serial.println(sensorData.pressure);
  Serial.print("Spannung: ");
  Serial.println(sensorData.voltage);
  Serial.println();
}

// Init Display ---------------------------------------------------------------------/
void initDisplay(){
// Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(3); // rotate display 90°, landscape mode
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_RED);
  delay(500);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
}

// Show time in minutes since last receive of data in upper right corner
void showMinutes(){
// fill upper right corner with black rectangle 
    tft.fillRect(140, 0 , 42, 14, ST77XX_BLACK); //x0,y0,w,h,colour
    // cursor for font baseline 2 pixels left and 12 pixels down  (140-2, 64+12)
    tft.setCursor(138, 12);  
    tft.setFont(&FreeMonoOblique9pt7b);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(minuteCountRaum);
    Serial.println(minuteCountRaum);
// fill lower right corner with red rectangle 
    tft.fillRect(140, 64 , 42, 14, ST77XX_RED); //x0,y0,w,h,colour
    // cursor for font baseline 2 pixels left and 12 pixels down  (140-2, 64+12)
    tft.setCursor(138, 76);
    tft.setFont(&FreeMonoOblique9pt7b);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(minuteCountLabor);
    Serial.println(minuteCountLabor);
}

// Show Data On Display -------------------------------------------------------------------/
void showDataOnDisplay(){
  // convert sensorData.location to string
  strLocation = sensorData.location;
  // output of string on serial interface to check 
  Serial.print("location: ");
  Serial.println(strLocation);
  // Y-cursor for font baseline
  int yCursor = 0;

 // show "Raum" temperature in upper part of display
  if (strLocation == "Raum"){
    // fill upper half of display with red rectangle 
    tft.fillRect(0, 0 , 160, 64, ST77XX_RED); //x0,y0,w,h,colour
    // cursor for font baseline 12 pixels below
    yCursor = 12; // 0 + 12
    // reset minute counter
    minuteCountRaum = 0;     
  }
  // show "Labor" temperature in lower part of display
  if (strLocation == "Labor"){
    // fill lower half of display with black rectangle
    tft.fillRect(0, 64 , 160, 64, ST77XX_BLACK); //x0,y0,w,h,colour
    // cursor for font baseline 12 pixels below
    yCursor = 76;  // 64 + 12
    // reset minute counter
    minuteCountLabor = 0;
  } 
  // set cursor on baseline of font
  tft.setCursor(0,yCursor);
  
  // write 1st line : the location
  tft.setFont(&FreeMonoOblique9pt7b);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(strLocation);
  
  // write 2nd line : the temperature and humidity
  yCursor = yCursor + 20; // cursor for font baseline 20 pixels below
  tft.setCursor(0,yCursor);
  tft.setFont(&FreeMonoOblique9pt7b);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(sensorData.temp);
  tft.drawCircle(64, yCursor-10, 2, 0x07E0); //draw circle instead of degree <°> sign
  tft.print(" C "); 
  tft.print(sensorData.humidity);
  tft.print("%");

  // write 3rd line : the voltage
  yCursor = yCursor + 16; // cursor for font baseline 16 pixels below
  tft.setCursor(0,yCursor);
  tft.setFont(&FreeMonoOblique9pt7b);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(sensorData.voltage);
  tft.print("V");
}

// Funktion "readVoltage" ------------------------------------------------------------------------------
void readVoltage(){
  valA0 = map(analogRead(A0),0,1023,0,55500); // read and scale analog value
  voltage = valA0;
  voltage = voltage / 10000;
}

// Funktion "readDHTSensor"-----------------------------------------------------------------------------
void readDHTSensor(){
    delay(20);
    temp = dht.readTemperature();
    humidity = dht.readHumidity();
    while (!(isValidTemp(temp) && isValidHumidity(humidity))) {
      delay(100);
      Serial.print("d"); // write "d" while waiting for valid data
      temp = dht.readTemperature();
      humidity = dht.readHumidity();
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
      Serial.print("b"); // write "b" while waiting for valid data
      temp = bme.readTemperature();
      humidity = bme.readHumidity();
      pressure = (bme.readPressure()/100.0F);
    } 
  }

// function: check if value humidity is valid ------------------------------------------ 
bool isValidHumidity(const double humidity) {
  return (!isnan(humidity) && humidity >= 0 && humidity <= 100);
}


// function: check if value temperature is valid --------------------------------------- 
bool isValidTemp(const double temp) {
  return (!isnan(temp) && temp >= -100 && temp <= 212);
}

// function: check if value pressure is valid ------------------------------------------ 
bool isValidPressure(const double pressure) {
  return (!isnan(pressure) && pressure >= -100 && pressure <= 1200);
}

// function "readLocalSensors" ---------------------------------------------------------
void readLocalSensors(){
  // read temperature and humidity from DHT
  if DHT_SENSOR {
    readDHTSensor(); } 
  // read temperature and humidity from BME
  if BME_SENSOR {
    bme.takeForcedMeasurement(); // has no effect in normal mode but in Weather monitoring
    readBMESensor();
  }
  // read Voltage
  readVoltage();
  // write "Raum" in receive-struct
  strcpy(sensorData.location, "Raum");
  // write temperature in receive-struct
  sensorData.temp = temp;
  // write humidity in receive-struct
  sensorData.humidity = humidity;
  // write voltage in receive-struct
  sensorData.voltage = voltage;
}

//------------------------------------------------------------------------------------/
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println();
  
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

  Serial.print("This node AP mac: "); Serial.println(WiFi.softAPmacAddress());
  Serial.print("This node STA mac: "); Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  initEspNow();

  // Initialize Display
  initDisplay();
}

//-----------------------------------------------------------------------------------/
void loop() {
  if (millis()-timer1 > 60000) {
    Serial.println("Waiting for ESP-NOW messages...");
    timer1 = millis();
    minuteCountRaum++;
    minuteCountLabor++;
    showMinutes();
  }
   if (millis()-timer2 > 30000) {
    Serial.println("Every 30sec. data request of built in sensor...");
    timer2 = millis();
    readLocalSensors();
    showDataOnDisplay();
  }
  if (NewDataReceived){
    NewDataReceived = false;
    // Print new data on serial interface
    PrintNewDataSerial();
    // Show new data on diplay
    showDataOnDisplay();
  }
}
