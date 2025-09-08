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

//#define SERIAL_RATE 115200
#define SERIAL_RATE 921600
//#define SERIAL_RATE 9600

//ROS
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

//MCP2515
#include <SPI.h>
#include <mcp2515.h>
#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// ROS VARIABLES
rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
std_msgs__msg__Float32 msg;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
esp_timer_handle_t periodic_timer;


IPAddress agent_ip(); 
size_t agent_port = 8888;


const char* SSID = "";
const char* password = "";


//MCP VARIABLES
struct can_frame canMsg1;
struct can_frame canMsg2;
MCP2515 mcp2515(10);
TaskHandle_t sendcanmsg;

void error_loop() {
  while(1) {
    Serial.println("There has been an error");
    delay(1000);
  }
}

static const char* TAG = "TimerExample";

void esp_timer_callback(void* arg) {
    int64_t start_time = esp_timer_get_time();  // in microseconds

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    int64_t end_time = esp_timer_get_time();  // in microseconds
    float delta_ms = (end_time - start_time) / 1000.0f;  // convert to milliseconds

    // Round to 3 decimal places
    msg.data = roundf(delta_ms * 1000.0f) / 1000.0f;

    //Serial.printf("Publish time: %.3f ms\n", msg.data);
}

void SendCANmsg(void* pvParameters){
  Serial.println("Running CAN msg 1 on ID xx");

  while(true){
    mcp2515.sendMessage(&canMsg1);
    delay(1000);
    Serial.print("Sent CAN message on ID: ");
    Serial.println("0x0F6");
  }
}

void setup() {

    delay(3000);
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(SERIAL_RATE);
    Serial.println("Started Serial...");
    delay(2000);
    // Print debug info BEFORE micro-ROS takes over Serial
    Serial.println("✅ Serial started");

    //MCP STARTUP
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

    xTaskCreatePinnedToCore(
    SendCANmsg,
    "CANMSG",
    10000,
    NULL,
    1,
    &sendcanmsg,
    1
    );

    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS);
    mcp2515.setNormalMode();



    //ROS STARTUP
    Serial.println("🚀 Starting micro-ROS WiFi setup...");

    msg.data = 0;

    WiFi.begin(SSID, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      Serial.print(".");
    }
    Serial.println("WiFi connected. IP Address:");
    Serial.println(WiFi.localIP());

    //check whether you have internet access
    Serial.println("Checking Agent Availability...");
    bool success = Ping.ping(agent_ip, 3);
    if(!success){
    Serial.println("Ping failed");
    return;
    }   
    Serial.println("Ping succesful.");

    char ssid[] = "";
    char psk[]= "";

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    delay(10000);

    Serial.println("Creating Node and Publisher...");
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/micro_ros_platformio_node_publisher"));
    Serial.println("Node and Publisher Created!");
    

    // Timer config
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &esp_timer_callback,
        .name = "my_periodic_timer"
    };


    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 5000)); // 1s = 1,000,000 us
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

}

void loop() {
  // Nothing needed here, everything is handled by the timer in the background
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5000)));
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);

}
