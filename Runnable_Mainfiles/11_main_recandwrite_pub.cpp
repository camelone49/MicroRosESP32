/*
    Name: Float32 Publisher
    Description: Publishes the time the publish function needed in the last iteration needed

    ROS:
        Node name: /micro_ros_platform_node
        Publishers: Float32 "/micro_ros_platformio_node_publisher"
        Subscribers: -
        Services: -
        Parameters: - 

*/
#include <Arduino.h>
#include "esp_timer.h"
#include "esp_log.h"

#include <WiFi.h>
#include <ESP32Ping.h>

#define SERIAL_RATE 115200
//#define SERIAL_RATE 921600
//#define SERIAL_RATE 9600

//ROS
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//#include <std_msgs/msg/int32.h>
//#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int16_multi_array.h>

//MCP2515
#include <SPI.h>
#include <mcp2515.h>
#define LED_PIN 2
#define MCP2515_INT_RX_PIN 16

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// ROS VARIABLES
rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
//std_msgs__msg__Float32 msg;

std_msgs__msg__Int16MultiArray msg;
int16_t can_data_array[10];


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
esp_timer_handle_t periodic_timer;

IPAddress agent_ip(); //brokk 5g HIER GANZ WICHTIG
size_t agent_port = 8888;


const char* SSID = "";
const char* password = "";


//MCP VARIABLES
struct can_frame canMsg1;
struct can_frame newCanMsg;
MCP2515 mcp2515(5);
//MCP2515 mcp2515_tx(17);

TaskHandle_t espStatus_handle;
volatile bool canMessageInterrupt = false;

void error_loop() {
  uint8_t loop_counter = 0;
  while (1) {
    Serial.printf("There has been an error (count = %d)\n", loop_counter);
    loop_counter++;
    delay(1000);
  }
}

static const char* TAG = "TimerExample";

void esp_timer_callback(void* arg) { 
  if (!canMessageInterrupt) {
    can_data_array[0] = (int16_t)(canMsg1.can_id & 0xFFFF);
    can_data_array[1] = canMsg1.can_dlc;
    for (int i = 0; i < canMsg1.can_dlc; i++) {
      can_data_array[2 + i] = canMsg1.data[i];
    }
    
    msg.data.size = 10;
    msg.data.capacity = 10;
    msg.data.data = can_data_array;
    
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  
    Serial.print(canMsg1.can_id, HEX);
    Serial.print("      ");
    Serial.print(canMsg1.can_dlc, HEX);
    Serial.print("      ");
    for (int i = 0; i < canMsg1.can_dlc; i++) {   // 🛠 Fixed newCanMsg -> canMsg1
      if (canMsg1.data[i] < 0x10) 
        Serial.print("0"); 
      Serial.print(canMsg1.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    
    canMessageInterrupt = false;   // 🔥 Clear only after publishing
  }

}

    

void ESPstatus(void* pvParameters){

  while(true){
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5000)));
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  }
}

void IRAM_ATTR canInterruptHandler() {
  //messageAlreadyForwarded = false;
  canMessageInterrupt = true;  // Set flag to signal new data
}

void setup() {

    delay(3000);
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(SERIAL_RATE);
    Serial.println("Started Serial...");
    delay(2000);
    // Print debug info BEFORE micro-ROS takes over Serial
    pinMode(MCP2515_INT_RX_PIN, INPUT);


    Serial.println("✅ Serial started");

    //MCP STARTUP
    Serial.println("Initializing CAN Program ...");

    mcp2515.reset();
    delay(1000);
    mcp2515.setBitrate(CAN_125KBPS,MCP_8MHZ);
    mcp2515.setNormalMode();

    //mcp2515_tx.reset();
    //mcp2515_tx.setBitrate(CAN_125KBPS);
    //mcp2515_tx.setNormalMode();

    //ROS STARTUP

      Serial.println("scan start");

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
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
    Serial.println("");
    Serial.println("🚀 Starting micro-ROS WiFi setup...");


    WiFi.begin(SSID, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      Serial.print(".");
    }
    Serial.println("WiFi connected. IP Address:");
    Serial.println(WiFi.localIP());
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());  // Print the SSID of the connected network

    //check whether you have internet access
    Serial.println("Checking Agent Availability...");
    bool success = Ping.ping(agent_ip, 3);
    if(!success){
    Serial.println("Ping failed");
    ESP.restart();
    return;
    }   
    Serial.println("Ping succesful.");

    char ssid[] = "";
    char psk[] = "";

    Serial.println("Setting up ROS agent....");
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    delay(10000);

    Serial.println("Creating Node and Publisher...");
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/micro_ros_platformio_node_publisher"));
    Serial.println("Node and Publisher Created!");

    delay(3000);
    

    // Timer config
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &esp_timer_callback,
        .name = "my_periodic_timer"
    };


    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer,1000)); // 1s = 1,000,000 us
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    xTaskCreatePinnedToCore(
    ESPstatus,
    "CANMSG",
    10000,
    NULL,
    1,
    &espStatus_handle,
    1
    );

  Serial.println("Running CAN msg 1 on ID xx");
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");

}

void loop() {
  // Nothing needed here, everything is handled by the timer in the background

    uint8_t irq = mcp2515.getInterrupts();  // Read the interrupt flags from MCP2515

    // Check if the RX0 buffer has a new message
    if (irq & MCP2515::CANINTF_RX0IF) {
        if (mcp2515.readMessage(MCP2515::RXB0, &canMsg1) == MCP2515::ERROR_OK) {
            // Print the CAN message received from RXB0
            Serial.println("Forwarding the message to the tx mcp.");
            //mcp2515_tx.sendMessage(&canMsg1);
        }
    }

    // Check if the RX1 buffer has a new message
    if (irq & MCP2515::CANINTF_RX1IF) {
        if (mcp2515.readMessage(MCP2515::RXB1, &canMsg1) == MCP2515::ERROR_OK) {
            // Print the CAN message received from RXB1
          delay(1000);
          Serial.println("I received a message over the other channel");
          delay(1000);
        }
    }
}



