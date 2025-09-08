#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>

// Define the CAN frame structure
struct can_frame frame;

// Instantiate the MCP2515 object, connecting to pin 5 for CS (Chip Select)
MCP2515 mcp2515(5);

// Volatile flag to indicate when an interrupt has occurred
volatile bool interrupt = false;

// Interrupt handler function, sets the interrupt flag
void irqHandler() {
    interrupt = true;
}

void setup() {
    // Start the serial communication for debugging
    Serial.begin(921600);
    delay(5000);  // Wait for the serial monitor to initialize

    Serial.println("Starting CAN Read Protocol");

    // Initialize the MCP2515 CAN controller
    mcp2515.reset();
    if (mcp2515.setBitrate(CAN_125KBPS) != MCP2515::ERROR_OK) {
        Serial.println("Failed to set bitrate");
        delay(1000);
    }
    mcp2515.setNormalMode();  // Set the MCP2515 to Normal mode for message reception

    // Print header for received messages
    Serial.println("------- CAN Read ----------");
    Serial.println("ID      DLC   DATA");

    // Attach an interrupt to pin 0 (INT pin of MCP2515)
    attachInterrupt(digitalPinToInterrupt(17), irqHandler, FALLING);
}

void loop() {
    // Check if an interrupt occurred
    if (interrupt) {
        interrupt = false;  // Reset the interrupt flag

        uint8_t irq = mcp2515.getInterrupts();  // Read the interrupt flags from MCP2515

        // Check if the RX0 buffer has a new message
        if (irq & MCP2515::CANINTF_RX0IF) {
            if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
                // Print the CAN message received from RXB0
                Serial.println("Received from RXB0");
                Serial.print(frame.can_id, HEX);  // Print CAN ID
                Serial.print("      ");
                Serial.print(frame.can_dlc, HEX); // Print DLC (Data Length Code)
                Serial.print("      ");

                // Print the CAN data byte by byte
                for (int i = 0; i < frame.can_dlc; i++) {
                    if (frame.data[i] < 0x10) 
                        Serial.print("0"); // Add leading zero
                    Serial.print(frame.data[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                delay(100);  // Optional delay for readability
            }
        }

        // Check if the RX1 buffer has a new message
        if (irq & MCP2515::CANINTF_RX1IF) {
            if (mcp2515.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK) {
                // Print the CAN message received from RXB1
                Serial.println("Received from RXB1");
                Serial.print(frame.can_id, HEX);  // Print CAN ID
                Serial.print("      ");
                Serial.print(frame.can_dlc, HEX); // Print DLC (Data Length Code)
                Serial.print("      ");

                // Print the CAN data byte by byte
                for (int i = 0; i < frame.can_dlc; i++) {
                    if (frame.data[i] < 0x10) 
                        Serial.print("0"); // Add leading zero
                    Serial.print(frame.data[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                delay(100);  // Optional delay for readability
            }
        }
    }
}
