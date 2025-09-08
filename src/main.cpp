/*
  Name: MCP2515 basic transmission code
  This is a file that can be flashed directly on to an ESP32 conntected to a MCP2515 board
  Hook up SPI comms and flash. Beforehand, put file in "src" folder and change name to "main.cpp"
*/


#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>

#define LED_PIN 2

struct can_frame canMsg1;
struct can_frame canMsg2;
MCP2515 mcp2515(10);
TaskHandle_t sendcanmsg;

//MULTITHREADING FUNCTION 1 ________________________________________________________________

void SendCANmsg(void* pvParameters){
  Serial.println("Running CAN msg 1 on ID xx");

  while(true){
    mcp2515.sendMessage(&canMsg1);
    delay(1000);
    Serial.print("Sent CAN message on ID: ");
    Serial.println("0x0F6");
  }
}


//SETUP FUNCTION ___________________________________________________________________________
void setup() {

  pinMode(LED_PIN, OUTPUT);
  delay(5000);
  Serial.begin(115200);
  Serial.println("Started the Serial Monitor");
  delay(5000);

  Serial.println("Initializing CAN Program ...");
  canMsg1.can_id  = 0x0F6;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x8E;
  canMsg1.data[1] = 0x87;
  canMsg1.data[2] = 0x32;
  canMsg1.data[3] = 0xFA;
  canMsg1.data[4] = 0x26;
  canMsg1.data[5] = 0x8E;
  canMsg1.data[6] = 0xBE;
  canMsg1.data[7] = 0x86;

  canMsg2.can_id  = 0x036;
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0x0E;
  canMsg2.data[1] = 0x00;
  canMsg2.data[2] = 0x00;
  canMsg2.data[3] = 0x08;
  canMsg2.data[4] = 0x01;
  canMsg2.data[5] = 0x00;
  canMsg2.data[6] = 0x00;
  canMsg2.data[7] = 0xA0;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  

  xTaskCreatePinnedToCore(
    SendCANmsg,
    "CANMSG",
    10000,
    NULL,
    1,
    &sendcanmsg,
    1
  );
  
  
  Serial.println("Example: Write to CAN");
  Serial.println("setup() running on core ");
  Serial.println(xPortGetCoreID());
}

// LOOP FUNCTION ______________________________________________________________
void loop() {

  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);

  delay(100);
}