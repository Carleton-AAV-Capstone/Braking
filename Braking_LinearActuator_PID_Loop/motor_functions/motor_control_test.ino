#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// #define TX_PIN 17
// #define RX_PIN 16
#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Target setpoint and integral value
int setpoint = 0;
float intval = 0;
const int maxPower = 3200;
float real_power = 0;
float power = 0;
float prev_error = 0;
unsigned long lastTime = 0;

// Parameters that can be changed on the fly
int P = 140;
int I = 0.5;
int D = 0;
int integralLimit = 1000;
int positionDeadzone = 1;
int errorLimit = 2048;

bool ros_enabled = true;

//MicroROS
rcl_subscription_t subscriber;
rcl_subscription_t subscriber_2;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
std_msgs__msg__Int32 msg_pub;

void error_loop(){
  while(1){
    delay(1000);
    Serial.println("ROS ERROR");
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  //Serial.println(msg->data);
  Serial.println("recv");
  if(msg != NULL){
  setPosition(msg->data);
  Serial.print("Recevied position: ");
  Serial.println(msg->data);
  }
}

// Setup function runs once when the program starts
void setup()
{
  Serial.begin(115200);
  // Reset the integral term to zero
  intval = 0;

  motor_controller_setup();
  if(ros_enabled){microRosSetup();}
  lastTime = millis();
}

void microRosSetup(){
  //MicroROS
  set_microros_wifi_transports("AAVwifi", "aav@2023", "192.168.1.126", 8888);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection failed");
    while (1); // Stop here if WiFi failed
  } else {
      Serial.println("WiFi connected");
      Serial.println(WiFi.localIP());
  }
  delay(2000);

  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "aav_braking_controller", "", &support));
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/brake"));

  // RCCHECK(rclc_subscription_init_default(
  //   &subscriber_2,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //   "/aav/braking/brake_pos_controller/set_controller_parameters"));

  // RCCHECK(rclc_publisher_init_best_effort(
  //   &publisher, 
  //   &node, 
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), 
  //   "/aav/braking/brake_pos_feedback");)

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_2, &msg, &subscription_callback, ON_NEW_DATA));
}

void setPIDconstraints(int p_val, int i_val, int d_val, int error_limit){
  P = p_val;
  I = i_val;
  D = d_val;
  errorLimit = error_limit;
}

void setPosition(int set){
  setpoint = set;
}

// Main loop runs continuously
void loop()
{
  if(!ros_enabled){
    if(Serial.available() > 0){
      setpoint = Serial.parseInt();
    }
  }
  // Read the current sensor value
  float brakePos = getA1_scaled();
  if(brakePos>1000){
    brakePos = 0;
  }
  // Get the time elapsed
  unsigned long currentTime = millis();
  //Calculate the elapsed time
  float elapsedTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float error = setpoint - brakePos;
  if(abs(error) < positionDeadzone){
    error = 0;
  }
  error = constrain(error, -errorLimit, errorLimit);

  // Calculate the integral term (accumulates the error over time)
  if(error != 0){
    intval += error * elapsedTime;
    intval = constrain(intval, -integralLimit, integralLimit);
  } else {
    intval = 0;
  }

  // Calculate the proportional term
  float propval = (error * P);

  // Calculate the integral term
  float ival = (intval * I);

  // Calculate the differential term
  float dval = (((error - prev_error)/elapsedTime) * D);
  prev_error = error;

  // Calculate the motor power based on proportional and integral control
  real_power = propval + ival + dval;
  power = real_power;
  // Constrain the power within the range of the 
  power = constrain(power, -maxPower, maxPower);
  // Set the motor speed based on the calculated power
  setMotorSpeed(power);

  // Output the current sensor value
  Serial.print("Pos:");
  Serial.print(brakePos);
  //Set point
  Serial.print(",Setpoint");
  Serial.print(setpoint);
  // Print the real power value
  Serial.print(",RealPower:");
  Serial.print(real_power);
  // Output the calculated motor power
  Serial.print(",Power:");
  Serial.println(power);

  if(ros_enabled){
    msg_pub.data = brakePos;
    // RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
  // delay(100);
}
