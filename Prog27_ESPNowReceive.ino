/**
 * ESP-NOW Receiver
 * 
 * This shows how to use an ESP8266/Arduino as an ESP-Now Gateway by having one
 * ESP8266 receive ESP-Now messages and write them to a TFT-display.
 * ESP8266 receive those messages over Serial and send them over WiFi. This is to
 * overcome the problem of ESP-Now not working at the same time as WiFi.
 * 
 * Author: Anthony Elder
 * License: Apache License v2
 */
 //**************************** Bibliotheken ******************************************/
#include <ESP8266WiFi.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Fonts/FreeSerif9pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>
#include <Fonts/FreeMonoBoldOblique9pt7b.h>
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeMonoOblique9pt7b.h>
#include <Fonts/FreeMonoOblique12pt7b.h>
#include <SPI.h>
extern "C" {
  #include <espnow.h>
  #include "user_interface.h"
}

//************************ Display *******************************************/

#define TFT_CS        D8
#define TFT_RST       D6
#define TFT_DC        D1
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
void setRotation(uint8_t rotation);

//************************Variablen*******************************************/
int heartBeat;
int minuteCountRaum = 0;
int minuteCountLabor = 0;
int count = 0;
bool Black = false;
//char* strRaum =""; //geht nicht
String  strRaum ="";
int monitoringTime;
volatile boolean NewDataReceived = false; // volatile Variable, weil sie während
                                          // Interrupt beinflusst wird

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

//*********************** Empfangs-Daten *************************************/
// keep in sync with slave struct
struct __attribute__((packed)) SENSOR_DATA {
  char Ort[16];
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

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  esp_now_register_recv_cb(OnDataRecv);
    // Investigate: There's little doc on what can be done within this method. If its like an ISR
    // then it should not take too long or do much I/O, but writing to Serial does appear to work ok
}


// Callback function that will be executed when data is received----------------------/
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  // Flag setzen: Neue Daten empfangen
  NewDataReceived = true;
}

// Neue Daten auf der seriellen Schnittstelle ausgeben--------------------------------/
void PrintNewDataSerial(){
  Serial.print("Char: ");
  Serial.println(sensorData.Ort);
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

// Anzeige Init ---------------------------------------------------------------------/
void Anzeige_Init(){
// Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(3); // Anzeige 90° drehen, landscape mode
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_RED);
  delay(500);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);

 // Erste Text-Ausgaben auf dem Display
  //tft.setTextColor(ST77XX_WHITE);
  //tft.setTextSize(3);
  //tft.println("START"); 
  return;
}

// Neue Daten auf Display ausgeben - einfach ---------------------------------------------/
void PrintNewDataOnDisplay_simple(){
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.print("Char: ");
  tft.println(sensorData.Ort);
  tft.print("Temperatur: ");
  tft.println(sensorData.temp);
  tft.print("Luftfeuchtigkeit: ");
  tft.println(sensorData.humidity);
  return;
}


// Minuten anzeigen seit dem letzten Empfang von Daten
void minuteAnzeige(){
// oben rechts mit schwarzem Rechteck ausfüllen 
    tft.fillRect(140, 0 , 42, 14, ST77XX_BLACK); //x0,y0,w,h,colour
    // Cursor auf Baseline der Schriftart um 12 Pixel nach unten setzen
    tft.setCursor(140, 12);  
    tft.setFont(&FreeMonoOblique9pt7b);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(minuteCountRaum);
    Serial.println(minuteCountRaum);
// unten rechts mit rotem Rechteck ausfüllen 
    tft.fillRect(140, 64 , 42, 14, ST77XX_RED); //x0,y0,w,h,colour
    // Cursor auf Baseline der Schriftart um 12 Pixel nach unten setzen
    tft.setCursor(140, 76);  
    tft.setFont(&FreeMonoOblique9pt7b);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(minuteCountLabor);
    Serial.println(minuteCountLabor);
}




// Anzeige -------------------------------------------------------------------/
void Anzeige(){
  // myData.Ort in String wandeln
  strRaum = sensorData.Ort;
  // String Kontrolle 
  Serial.print("Raum: ");
  Serial.println(strRaum);
  // Y-Cursor für Schrift Baseline
  int yCursor = 0;

 // Raum Temperatur oben anzeigen
 // (strRaum == "Labor" && count == 1)
  if (strRaum == "Raum"){
    // obere Displayhälfte mit rotem Rechteck ausfüllen 
    tft.fillRect(0, 0 , 192, 64, ST77XX_RED); //x0,y0,w,h,colour
    // Cursor auf Baseline der Schriftart um 12 Pixel nach unten setzen
    yCursor = 12; // 0 + 12
    // Minutenzähler rücksetzen
    minuteCountRaum = 0;     
  }
  // Labor Temperatur unten anzeigen
  // ((strRaum == "Labor") && (count == 2))
  if (strRaum == "Labor"){
    // untere Displayhälfte mit schwarzem Rechteck ausfüllen 
    tft.fillRect(0, 64 , 192, 64, ST77XX_BLACK); //x0,y0,w,h,colour
    // Cursor auf Baseline der Schriftart um 12 Pixel nach unten setzen
    yCursor = 76;  // 64 + 12
    // Minutenzähler rücksetzen
    minuteCountLabor = 0;
  } 
  // Cursor auf Baseline der Schriftart setzen
  tft.setCursor(0,yCursor);
  
  // 1. Zeile schreiben : den Ort
  tft.setFont(&FreeMonoOblique9pt7b);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(strRaum);
  
  // 2. Zeile schreiben : die Temperatur
  yCursor = yCursor + 20; // Baseline der Schrift um 20 nach unten setzen bei 18er Schrift
  tft.setCursor(0,yCursor);
  tft.setFont(&FreeMonoOblique9pt7b);
  //tft.setTextSize(3);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(sensorData.temp);
  tft.drawCircle(64, yCursor-10, 2, 0x07E0); //Grad-Zeichen
  tft.print(" C "); 
  tft.print(sensorData.humidity);
  tft.print("%");

  // 3.Zeile schreiben : die Spannung
  yCursor = yCursor + 16; // Baseline der Schrift um 16 nach unten setzen bei 12er Schrift
  tft.setCursor(0,yCursor);
  tft.setFont(&FreeMonoOblique9pt7b);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(sensorData.voltage);
  tft.print("V");

  // Zähler rücksetzen
  if (count >=2){
    count = 0;
  }
}

//------------------------------------------------------------------------------------/
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println();

  Serial.print("This node AP mac: "); Serial.println(WiFi.softAPmacAddress());
  Serial.print("This node STA mac: "); Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  initEspNow();

  // Display initialisieren
  Anzeige_Init();
}

//-----------------------------------------------------------------------------------/
void loop() {
  if (millis()-heartBeat > 60000) {
    Serial.println("Waiting for ESP-NOW messages...");
    heartBeat = millis();
    minuteCountRaum++;
    minuteCountLabor++;
    minuteAnzeige();
  }
  if (NewDataReceived){
    NewDataReceived = false;
    // Daten auf der seriellen Schnittstelle ausgeben
    PrintNewDataSerial();
    //Daten auf Display anzeigen
    //PrintNewDataOnDisplay_simple();
    Anzeige();
  }
}
