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


#define SERIAL_RATE 115200
//#define SERIAL_RATE 921600
//#define SERIAL_RATE 9600
#define RMW_UXRCE_TRANSPORT_SERIAL 1 

//ROS
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
std_msgs__msg__Float32 msg;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
esp_timer_handle_t periodic_timer;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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



void setup() {
    Serial.begin(SERIAL_RATE);
    Serial.println("Started Serial...");
    delay(2000);
    // Print debug info BEFORE micro-ROS takes over Serial
    Serial.println("✅ Serial started");
    Serial.println("🚀 Starting micro-ROS setup...");


    msg.data = 0;

    
    set_microros_serial_transports(Serial);
    delay(10000);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/micro_ros_platformio_node_publisher"));

    

    // Timer config
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &esp_timer_callback,
        .name = "my_periodic_timer"
    };


    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 500000)); // 1s = 1,000,000 us
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

}

void loop() {
    // Nothing needed here, everything is handled by the timer in the background
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5000)));

}
