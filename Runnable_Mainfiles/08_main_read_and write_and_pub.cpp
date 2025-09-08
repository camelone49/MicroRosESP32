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

//#include <std_msgs/msg/int32.h>
//#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int16_multi_array.h>

//MCP2515
#include <SPI.h>
#include <mcp2515.h>
#define LED_PIN 2
#define MCP2515_INT_RX_PIN 4

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

//IPAddress agent_ip(172,20,10,7); //brokk
//IPAddress agent_ip(192, 168, 1, 167); //hiphne
//IPAddress agent_ip(10,0,43,55); //FAHO network
IPAddress agent_ip(192,168,0,101); //TP Link Router
size_t agent_port = 8888;

//const char* SSID = "FAHO Classic";
//const char* password = "Mzde-o3pn-jwF1-SHUP-3zXa";

//const char* SSID = "HiPhone (2)";
//const char* password = "hisham04";

//const char* SSID = "Router_F5AF08_2.4G_Jetson";
//const char* password = "innok444";

const char* SSID = "TP-Link_E06C";
const char* password = "97083641";


//MCP VARIABLES
struct can_frame canMsg1;
MCP2515 mcp2515(5);
MCP2515 mcp2515_tx(17);

TaskHandle_t espStatus_handle;
volatile bool canMessageReady = false;


//Create a mutex for thread safety
SemaphoreHandle_t canMsgMutex;


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

if (xSemaphoreTake(canMsgMutex, 0) == pdTRUE) {
  can_data_array[0] = (int16_t)(canMsg1.can_id & 0xFFFF);
  can_data_array[1] = canMsg1.can_dlc;
  for (int i = 0; i < 8; i++) {
    can_data_array[2 + i] = canMsg1.data[i];
  }
  xSemaphoreGive(canMsgMutex);
}

  msg.data.size = 10;
  msg.data.capacity = 10;
  msg.data.data = can_data_array;

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

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
  canMessageReady = true;
}

void setup() {

    delay(3000);
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(SERIAL_RATE);
    Serial.println("Started Serial...");
    delay(2000);
    // Print debug info BEFORE micro-ROS takes over Serial
    canMsgMutex = xSemaphoreCreateMutex();
    pinMode(MCP2515_INT_RX_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MCP2515_INT_RX_PIN), canInterruptHandler, FALLING);


    if (canMsgMutex == NULL) {
      Serial.println("❌ Failed to create CAN message mutex!");
      error_loop();
    }
    Serial.println("✅ Serial started");

    //MCP STARTUP
    Serial.println("Initializing CAN Program ...");

    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS);
    mcp2515.setNormalMode();

    //ROS STARTUP
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

    //char ssid[] = "Faho Classic";
    //char psk[]= "Mzde-o3pn-jwF1-SHUP-3zXa";

    //char ssid[] = "HiPhone (2)";
    //char psk[]= "hisham04";

    //char ssid[] = "Router_F5AF08_2.4G_Jetson";
    //char psk[] = "innok444";

    char ssid[] = "TP-Link_E06C";
    char psk[] = "97083641";

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
    

    // Timer config
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &esp_timer_callback,
        .name = "my_periodic_timer"
    };


    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000)); // 1s = 1,000,000 us
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



}
