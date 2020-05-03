# ESP-NOW
ESP-NOW with ESP8266, Weather sensor sends data to ESP8266 receiver. Data are displayed at TFT screen.
The following ESP-NOW programs are based on the work of HarringayMakerspace/ESP-Now.
https://github.com/HarringayMakerSpace/ESP-Now.
My programs are adapted to a weatherstation and a TFT-Display.

This respository consists of 2 programs.

1. Prog28_ESPNowSend:
This program has the possibility to get data either from a DHT22 sensor or a BME280 sensor.
The weather data are written in the struct SENSOR_DATA, which is transmitted
via the ESP-NOW protocol to the receiver every 6 minutes.
Between the transmissions the ESP is going into deep sleep.

2. Prog27_ESPNowReceive:
This programs receives the SENSOR_DATA and displays the data on a
TFT-screen.
