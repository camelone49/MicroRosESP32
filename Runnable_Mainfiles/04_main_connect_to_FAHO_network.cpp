/*
 *  ESP32 Get MAC Address Example
 *  Full Tutorial @ https://deepbluembedded.com/esp32-wifi-library-examples-tutorial-arduino/
 * 
 *  Name: FAHO Wifi Connector
 *  This code connects an ESP32 to the FAHO wifi network
 * 
 */
#include <WiFi.h>
#include <Arduino.h>

const char* ssid = "";
const char* password = "";

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}
 
void setup(){

  Serial.begin(115200);
  Serial.println("Starting the ESP32 basic wifi setup");
  delay(5000);
  Serial.print("\nDefault ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("Setup done. Starting Scan for nearby networks");


  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("Scan done");
  if (n == 0) {
      Serial.println("no networks found");
  } 
  else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10);
    }
  }
    
  Serial.println("Scanning Complete. Connecting to FAHO Network...");
  initWiFi();


}
 
void loop(){

  if (WiFi.status() == WL_CONNECTED){
    Serial.println("ESP32 is still connected");
  } 
  else {
    Serial.println("ESP is disconnected");
  }

  delay(5000);
}