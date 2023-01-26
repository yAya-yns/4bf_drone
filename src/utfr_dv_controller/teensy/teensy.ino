#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <utfr_msgs/msg/teensy.h>
#include <utfr_msgs/msg/system_status.h>

// Define variables:

// Params
const unsigned int timer_timeout_ = 100; //[ms]

// Publishers and Subscribers
rcl_publisher_t teensy_publisher_;
rcl_subscription_t system_status_subscriber_;
utfr_msgs__msg__Teensy teensy_msg_;
utfr_msgs__msg__SystemStatus status_;

// RCLCPP essentials
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&teensy_publisher_, &teensy_msg_, NULL));
  }
  if (teensy_msg_.gonogo == true){
    teensy_msg_.gonogo = false;
  }
  else {
    teensy_msg_.gonogo = true;
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &teensy_publisher_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(utfr_msgs, msg, Teensy),
    "/teensy"));

  // create timer,
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout_),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  teensy_msg_.gonogo = false;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
