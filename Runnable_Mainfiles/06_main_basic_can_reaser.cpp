#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>

struct can_frame canMsg;
MCP2515 mcp2515(5);


void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Starting CAN Read Protocoll");
  
  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_125KBPS) != MCP2515::ERROR_OK) {
    Serial.println("Failed to set bitrate");
    delay(1000);
  }
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print("      "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print("      ");
    
    for (int i = 0; i < canMsg.can_dlc; i++) {
      if (canMsg.data[i] < 0x10) Serial.print("0"); // Add leading zero
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }
    delay(100);
    Serial.println(); // Add this after the for-loop

  }
}
