#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

#include <MS5837.h>

#include <ADS1X15.h>

#include <LibAlg.hpp>
#include <PID.hpp>
#include <ThrusterSolver.hpp>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <rsla_interfaces/msg/euler_angles.h>
#include <rsla_interfaces/msg/pose_euler.h>
#include <rsla_interfaces/msg/pid.h>
#include <rsla_interfaces/msg/pose_with_mask.h>
#include <rsla_interfaces/msg/wrench_with_mask.h>
#include <rsla_interfaces/msg/controller_telemetry.h>

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while(0)\

//#define DEBUG

// Hardware objects
constexpr uint8_t LED_PIN = 13;

constexpr uint8_t BALL_DROPPER_PIN = 12;

Servo ball_dropper;

// Thruster Allocation                                 
LA::FloatMatrix allocation_matrix = {
    {     0.0,    0.707,      0.0,   -0.707,      0.0,    0.707,      0.0,   -0.707},
    {     0.0,   -0.707,      0.0,   -0.707,      0.0,    0.707,      0.0,    0.707},
    {    -1.0,      0.0,     -1.0,      0.0,     -1.0,      0.0,     -1.0,      0.0},
    {  -0.385,  -0.0417,   -0.385,  -0.0417,    0.385,   0.0417,    0.385,   0.0417},
    {   0.237,  -0.0417,   -0.219,   0.0417,    0.237,  -0.0417,   -0.219,   0.0417},
    {     0.0,  -0.3153,     -0.0,   0.3026,      0.0,   0.3153,      0.0,  -0.3026}
};

// State objects
enum class VehicleState : int8_t
{
  ADC_ERROR = -4,
  PWM_ERROR = -3,
  BARO_ERROR = -2,
  IMU_ERROR = -1,
  IDLE = 0,
  ARMED = 1
} vehicle_state;

enum class NodeState : int8_t
{
  WAITING_AGENT = 0,
  AGENT_AVAILABLE = 1,
  AGENT_CONNECTED = 2,
  AGENT_DISCONNECTED = 3
} node_state;

bool startup_successful = true;
bool vehicle_armed = false;

float x = 0;
float y = 0;
float z = 0;

float yaw = 0;
float pitch = 0;
float roll = 0;

// ROS objects

// Status publisher
rcl_publisher_t status_publisher;
std_msgs__msg__Int8 status_msg;
rcl_timer_t status_timer;
const unsigned int status_timer_interval_ms = 500; // 2 Hz

// Orientation publisher
rcl_publisher_t orientation_publisher;
rsla_interfaces__msg__PoseEuler orientation_msg;
rcl_timer_t orientation_timer;
const unsigned int orientation_timer_interval_ms = 100; // 10 Hz

// Telemetry publisher
rcl_publisher_t telemetry_publisher;
rsla_interfaces__msg__ControllerTelemetry telemetry_msg;
rcl_timer_t telemetry_timer;
const unsigned int telemetry_timer_interval_ms = 250; // 4 Hz

// Battery voltage publisher
rcl_publisher_t battery_voltage_publisher;
std_msgs__msg__Float32 battery_voltage_msg;
rcl_timer_t battery_voltage_timer;
const unsigned int battery_voltage_interval_ms = 1000; // 1 Hz

// Ping publisher
rcl_publisher_t ping_publisher;
std_msgs__msg__Int32 ping_msg;
rcl_timer_t ping_timer;

// Setpoint subscribers
rcl_subscription_t pose_setpoint_subscriber;
rsla_interfaces__msg__PoseWithMask pose_setpoint_msg;

rcl_subscription_t wrench_setpoint_subscriber;
rsla_interfaces__msg__WrenchWithMask wrench_setpoint_msg;

// Tuning subscriber
rcl_subscription_t pid_tuning_subscriber;
rsla_interfaces__msg__PID pid_tuning_msg;

// Arming subscriber
rcl_subscription_t sw_arm_subscriber;
std_msgs__msg__Bool sw_arm_msg;

// Ball Dropper subscriber
rcl_subscription_t ball_dropper_subscriber;
std_msgs__msg__Int8 ball_dropper_msg;

// Diagnostics subscriber
rcl_subscription_t diagnostic_command_subscriber;
std_msgs__msg__Int8 diagmonst_command_msg;

// ROS node
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Sensor objects
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
imu::Vector<3> bno_euler_data;
uint8_t bno_sys_cal, bno_gyro_cal, bno_acc_cal, bno_mag_cal = 0;

MS5837 baro(0x76, &Wire);
float baro_depth_data = 0;
float surface_offset = 0;

Adafruit_PWMServoDriver pwm_driver(0x40);

ADS1113 ADS(0x48);

float battery_voltage = 16.8;

// Controller objects
enum class ControllerMode : uint8_t
{
  EFFORT = 0,
  POSITION = 1
};

ControllerMode x_mode = ControllerMode::EFFORT;
ControllerMode y_mode = ControllerMode::EFFORT;
ControllerMode z_mode = ControllerMode::POSITION;
ControllerMode roll_mode = ControllerMode::POSITION;
ControllerMode pitch_mode = ControllerMode::POSITION;
ControllerMode yaw_mode = ControllerMode::POSITION;

RSLA::PID x_controller(0, 0, 0);
RSLA::PID y_controller(0, 0, 0);
RSLA::PID z_controller(0, 0, 0);
RSLA::PID roll_controller(0, 0, 0);
RSLA::PID pitch_controller(0, 0, 0);
RSLA::PID yaw_controller(0, 0, 0);

float x_effort = 0;
float y_effort = 0;
float z_effort = 0;
float roll_effort = 0;
float pitch_effort = 0;
float yaw_effort = 0;

float control_vector[6];
float thruster_forces[8];
uint16_t thruster_pwm[8];

RSLA::ThrusterSolver thruster_solver(allocation_matrix);

// Timing objects
uint32_t last_loop_ms = 0;

uint32_t last_pwm_write_ms = 0;
uint32_t pwm_write_interval_ms = 50; // 20Hz

uint32_t last_sensor_update_ms = 0;
uint32_t sensor_update_interval_ms = 50; // 20Hz
bool new_sensor_data = false;


//function declarations
void error_loop();
void status_callback(rcl_timer_t* timer, int64_t last_call_time);
void orientation_callback(rcl_timer_t* timer, int64_t last_call_time);
void telemetry_callback(rcl_timer_t* timer, int64_t last_call_time);
void battery_voltage_callback(rcl_timer_t* timer, int64_t last_call_time);
void ping_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void pose_setpoint_callback(const void *msgin);
void wrench_setpoint_callback(const void *msgin);
void pid_tuning_callback(const void *msgin);
void sw_arm_callback(const void *msgin);
void diagnostic_command_callback(const void *msgin);
bool create_entities();
void destroy_entities();
void initialize_sensors();
void initialize_controllers();
void initialize_message_data();
void setup();
void get_imu();
void get_baro();
void get_battery();
void update_pids(float dt);
void update_control_vector();
void update_output_vectors();
void control_ball_dropper();
void output_to_pwm(uint32_t loop_time_millis);

void loop() {
  // Loop timing
  uint32_t loop_time_millis = millis();
  float dt = (float)(last_loop_ms - loop_time_millis) / 1000.f;
  last_loop_ms = loop_time_millis;

  if(node_state == NodeState::AGENT_CONNECTED)
  {
    digitalWriteFast(LED_PIN, loop_time_millis % 2000 > 1000);
  }
  else
  {
    digitalWriteFast(LED_PIN, 0);
  }

  // Main loop code
  if(startup_successful)
  {
    if(loop_time_millis > last_sensor_update_ms + sensor_update_interval_ms)
    {
      last_sensor_update_ms = loop_time_millis;

      get_imu();
      get_baro();
      get_battery();

      new_sensor_data = true;
    }
    
    if(new_sensor_data)
    {
      update_pids(dt);
      new_sensor_data = false;
    }

    update_control_vector();

    // Update thruster solution
    thruster_solver.solve(control_vector);

    update_output_vectors(); 
    control_ball_dropper();
    output_to_pwm(loop_time_millis);
  }

  // Spin ROS node
  switch(node_state)
  {
    case NodeState::WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, node_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1) ? NodeState::AGENT_AVAILABLE : NodeState::WAITING_AGENT));
      break;
    case NodeState::AGENT_AVAILABLE:
      node_state = (true == create_entities()) ? NodeState::AGENT_CONNECTED : NodeState::WAITING_AGENT;
      break;
    case NodeState::AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, node_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1) ? NodeState::AGENT_CONNECTED : NodeState::AGENT_DISCONNECTED));
      if(node_state == NodeState::AGENT_CONNECTED)
      {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
      }
      break;
    case NodeState::AGENT_DISCONNECTED:
      destroy_entities();
      node_state = NodeState::WAITING_AGENT;
      break;
    default:
      break;
  }
}




//-----------Functions--------------------------------------------------------

void setup() {
  Wire.begin();

  initialize_sensors();
  
  // Reset i2c clock after sensor startup
  Wire.setClock(400000);

  initialize_controllers();

  // Initialize MicroROS transport
  Serial.begin(1000000); // This doesn't actually care about the number
  set_microros_serial_transports(Serial);
  
  while(!Serial) delay(10); // Loop until serial established

  // Set up error indicators
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(1000);
  
  initialize_message_data();

  // Initial state
  node_state = NodeState::WAITING_AGENT;
}

// Eternal loop for unrecoverable errors  
void error_loop()
{
  while(1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Status heartbeat timer callback
void status_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if(timer != NULL)
  {
    int8_t status = 0;

    switch(vehicle_state)
    {
      case VehicleState::PWM_ERROR:
        status = -3;
        break;
      case VehicleState::BARO_ERROR:
        status = -2;
        break;
      case VehicleState::IMU_ERROR:
        status = -1;
        break;
      case VehicleState::IDLE:
        status = 0;
        break;
      case VehicleState::ARMED:
        status = 1;
        break;
      default:
        status = -4;
        break;
    }

    status_msg.data = status;
    RCSOFTCHECK(rcl_publish(&status_publisher, &status_msg, NULL));
  }
}

// Orientation timer callback
void orientation_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if(timer != NULL)
  {
    orientation_msg.orientation.yaw = yaw;
    orientation_msg.orientation.pitch = pitch;
    orientation_msg.orientation.roll = roll;

    orientation_msg.position.z = z;

    RCSOFTCHECK(rcl_publish(&orientation_publisher, &orientation_msg, NULL));
  }
}

// Telemetry timer callback
void telemetry_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if(timer != NULL)
  {
    telemetry_msg.controller_modes[0] = (uint8_t)(x_mode == ControllerMode::POSITION);
    telemetry_msg.controller_modes[1] = (uint8_t)(y_mode == ControllerMode::POSITION);
    telemetry_msg.controller_modes[2] = (uint8_t)(z_mode == ControllerMode::POSITION);
    telemetry_msg.controller_modes[3] = (uint8_t)(roll_mode == ControllerMode::POSITION);
    telemetry_msg.controller_modes[4] = (uint8_t)(pitch_mode == ControllerMode::POSITION);
    telemetry_msg.controller_modes[5] = (uint8_t)(yaw_mode == ControllerMode::POSITION);

    memcpy(&telemetry_msg.control_vector, &control_vector, 6 * sizeof(float));
    memcpy(&telemetry_msg.thruster_vector, &thruster_forces, 8 * sizeof(float));
    memcpy(&telemetry_msg.pwm_output, &thruster_pwm, 8 * sizeof(uint16_t));

    telemetry_msg.imu_calibration[0] = bno_sys_cal;
    telemetry_msg.imu_calibration[1] = bno_gyro_cal;
    telemetry_msg.imu_calibration[2] = bno_acc_cal;
    telemetry_msg.imu_calibration[3] = bno_mag_cal;

    RCSOFTCHECK(rcl_publish(&telemetry_publisher, &telemetry_msg, NULL));
  }
}

// Battery voltage timer callback
void battery_voltage_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if(timer != NULL)
  {
    battery_voltage_msg.data = battery_voltage;

    RCSOFTCHECK(rcl_publish(&battery_voltage_publisher, &battery_voltage_msg, NULL));
  }
}

// Ping timer callback
void ping_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  
  if(timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&ping_publisher, &ping_msg, NULL));
    ping_msg.data++;
  }
}

// Pose setpoint callback
void pose_setpoint_callback(const void *msgin)
{
  const rsla_interfaces__msg__PoseWithMask *msg = (const rsla_interfaces__msg__PoseWithMask*)msgin;
  
  // X controller
  if(msg->mask & 1)
  {
    x_mode = ControllerMode::POSITION;
    x_controller.setpoint = msg->cmd.position.x;
  }
  // Y controller
  if(msg->mask & 2)
  {
    y_mode = ControllerMode::POSITION;
    y_controller.setpoint = msg->cmd.position.y;
  }
  // Z controller
  if(msg->mask & 4)
  {
    z_mode = ControllerMode::POSITION;
    z_controller.setpoint = msg->cmd.position.z;
  }

  // Yaw controller
  if(msg->mask & 8)
  {
    roll_mode = ControllerMode::POSITION;
    roll_controller.setpoint = msg->cmd.orientation.roll;
  }
  // Pitch controller
  if(msg->mask & 16)
  {
    pitch_mode = ControllerMode::POSITION;
    pitch_controller.setpoint = msg->cmd.orientation.pitch;
  }
  // Roll controller
  if(msg->mask & 32)
  {
    yaw_mode = ControllerMode::POSITION;
    yaw_controller.setpoint = msg->cmd.orientation.yaw;
  }
}

// Wrench setpoint callback
void wrench_setpoint_callback(const void *msgin)
{
  const rsla_interfaces__msg__WrenchWithMask *msg = (const rsla_interfaces__msg__WrenchWithMask*)msgin;
  
  // X controller
  if(msg->mask & 1)
  {
    x_mode = ControllerMode::EFFORT;
    x_effort = msg->cmd.force.x;
  }
  // Y controller
  if(msg->mask & 2)
  {
    y_mode = ControllerMode::EFFORT;
    y_effort = msg->cmd.force.y;
  }
  // Z controller
  if(msg->mask & 4)
  {
    z_mode = ControllerMode::EFFORT;
    z_effort = msg->cmd.force.z;
  }

  // Yaw controller
  if(msg->mask & 8)
  {
    roll_mode = ControllerMode::EFFORT;
    roll_effort = msg->cmd.torque.x;
  }
  // Pitch controller
  if(msg->mask & 16)
  {
    pitch_mode = ControllerMode::EFFORT;
    pitch_effort = msg->cmd.torque.y;
  }
  // Roll controller
  if(msg->mask & 32)
  {
    yaw_mode = ControllerMode::EFFORT;
    yaw_effort = msg->cmd.torque.z;
  }
}

// Tuning setpoint callback
void pid_tuning_callback(const void *msgin)
{
  const rsla_interfaces__msg__PID *msg = (const rsla_interfaces__msg__PID*)msgin;

  uint8_t pid_index = msg->pid;

  float k_p = msg->kp;
  float k_i = msg->ki;
  float k_d = msg->kd;

  float antiwindup = msg->antiwindup;
  bool enable_antiwindup = antiwindup > 0;
  
  float bias = msg->bias;

  float constraint = msg->constraint;
  bool enable_constraint = constraint > 0;

  RSLA::PID *controller;

  switch(pid_index)
  {
    case 0:
      controller = &x_controller;
      break;
    case 1:
      controller = &y_controller;
      break;
    case 2:
      controller = &z_controller;
      break;
    case 3:
      controller = &roll_controller;
      break;
    case 4:
      controller = &pitch_controller;
      break;
    case 5:
      controller = &yaw_controller;
      break;
    default:
      return;
  }

  controller->kP = k_p;
  controller->kI = k_i;
  controller->kD = k_d;
  controller->antiwindup = antiwindup;
  controller->enableAntiwindup = enable_antiwindup;
  controller->bias = bias;
  controller->constraint = constraint;
  controller->enableConstraint = enable_constraint;
}

// Armed callback
void sw_arm_callback(const void *msgin)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool*)msgin;

  // Do some pre"flight" checks in the future here
  // But for now just set the armed state

  bool new_arm = msg->data;
  
  // If vehicle previously disarmed, reset PID integrals
  if(new_arm && !vehicle_armed)
  {
    x_controller.zeroIntegrator();
    y_controller.zeroIntegrator();
    z_controller.zeroIntegrator();
    roll_controller.zeroIntegrator();
    pitch_controller.zeroIntegrator();
    yaw_controller.zeroIntegrator();
  }

  vehicle_armed = new_arm;
}

// Diagnostic command callback
void diagnostic_command_callback(const void *msgin)
{
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8*)msgin;

  // Miscellatious diagnostic commands
  switch(msg->data)
  {
    case 1:
      // Reset surface offset
      surface_offset = baro_depth_data;
      break;
    default:
      break;
  }
}

// Create ROS entities
bool create_entities()
{
  // Create ROS node
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))
  RCCHECK(rclc_node_init_default(&node, "rsla_controls_node", "", &support));

  // Create status publisher
  RCCHECK(rclc_publisher_init_default(&status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "rsla/controls/status"));
  RCCHECK(rclc_timer_init_default(&status_timer, &support, RCL_MS_TO_NS(status_timer_interval_ms), status_callback));

  // Create orientation publisher
  RCCHECK(rclc_publisher_init_default(&orientation_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rsla_interfaces, msg, PoseEuler), "rsla/controls/pose_euler"));
  RCCHECK(rclc_timer_init_default(&orientation_timer, &support, RCL_MS_TO_NS(orientation_timer_interval_ms), orientation_callback));

  // Create telemetry publisher
  RCCHECK(rclc_publisher_init_default(&telemetry_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rsla_interfaces, msg, ControllerTelemetry), "rsla/controls/debug_telemetry"));
  RCCHECK(rclc_timer_init_default(&telemetry_timer, &support, RCL_MS_TO_NS(telemetry_timer_interval_ms), telemetry_callback));

  // Create battery voltage publisher
  RCCHECK(rclc_publisher_init_default(&battery_voltage_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "rsla/controls/battery_voltage"));
  RCCHECK(rclc_timer_init_default(&battery_voltage_timer, &support, RCL_MS_TO_NS(battery_voltage_interval_ms), battery_voltage_callback));

  // Create setpoint subscribers
  RCCHECK(rclc_subscription_init_default(&pose_setpoint_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rsla_interfaces, msg, PoseWithMask), "rsla/autonomy/pose_setpoint_with_mask"));
  RCCHECK(rclc_subscription_init_default(&wrench_setpoint_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rsla_interfaces, msg, WrenchWithMask), "rsla/autonomy/wrench_setpoint_with_mask"));

  // Create tuning subscriber
  RCCHECK(rclc_subscription_init_default(&pid_tuning_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rsla_interfaces, msg, PID), "rsla/controls/pid_controller_gains"));

  // Create software arm subscriber
  RCCHECK(rclc_subscription_init_default(&sw_arm_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "rsla/controls/sw_arm"));

  // Create ball dropper subscriber
  RCCHECK(rclc_subscription_init_default(&ball_dropper_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "rsla/controls/ball_dropper"));

  // Create diagnostic command subscriber
  RCCHECK(rclc_subscription_init_default(&diagnostic_command_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "rsla/controls/diagnostic_command"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator));

  // Add timers to executor
  RCCHECK(rclc_executor_add_timer(&executor, &status_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &orientation_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &telemetry_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &battery_voltage_timer));

  // Add subscribers to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &pose_setpoint_subscriber, &pose_setpoint_msg, &pose_setpoint_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &wrench_setpoint_subscriber, &wrench_setpoint_msg, &wrench_setpoint_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pid_tuning_subscriber, &pid_tuning_msg, &pid_tuning_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sw_arm_subscriber, &sw_arm_msg, &sw_arm_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &diagnostic_command_subscriber, &diagmonst_command_msg, &diagnostic_command_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  RCL_UNUSED(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  RCCHECK(rcl_publisher_fini(&status_publisher, &node));
  RCCHECK(rcl_publisher_fini(&orientation_publisher, &node));
  RCCHECK(rcl_publisher_fini(&telemetry_publisher, &node));
  RCCHECK(rcl_publisher_fini(&battery_voltage_publisher, &node));

  RCCHECK(rcl_timer_fini(&status_timer));
  RCCHECK(rcl_timer_fini(&orientation_timer));
  RCCHECK(rcl_timer_fini(&telemetry_timer));
  RCCHECK(rcl_timer_fini(&battery_voltage_timer));

  RCCHECK(rcl_subscription_fini(&pose_setpoint_subscriber, &node));
  RCCHECK(rcl_subscription_fini(&wrench_setpoint_subscriber, &node));
  RCCHECK(rcl_subscription_fini(&pid_tuning_subscriber, &node));
  RCCHECK(rcl_subscription_fini(&sw_arm_subscriber, &node));
  RCCHECK(rcl_subscription_fini(&diagnostic_command_subscriber, &node));

  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));
}

void initialize_sensors() {
  if(!baro.init())
  {
    vehicle_state = VehicleState::BARO_ERROR;
    startup_successful = false;
  }
  else
  {
    baro.setFluidDensity(997);
  }

  if(!bno.begin())
  {
    vehicle_state = VehicleState::IMU_ERROR;
    startup_successful = false;
  }
  else
  {
    bno.setMode(OPERATION_MODE_IMUPLUS);
    bno.setExtCrystalUse(true);
  }

  delay(100);

  if(!pwm_driver.begin())
  {
    vehicle_state = VehicleState::PWM_ERROR;
    startup_successful = false;
  }
  else
  {
    pwm_driver.setPWMFreq(250);
    pwm_driver.setOscillatorFrequency(25600000);
  }

  delay(100);

  if(!ADS.begin())
  {
    vehicle_state = VehicleState::ADC_ERROR;
    startup_successful = false;
  }
  else
  {
    ADS.setDataRate(4);
    ADS.setMode(0);
    ADS.readADC(0);
  }

}

void initialize_controllers() {
  yaw_controller.errorMode = RSLA::ErrorMode::ANGULAR;

  x_controller.derivativeMode = RSLA::DerivativeMode::DERIVATIVE_ON_MEASUREMENT;
  y_controller.derivativeMode = RSLA::DerivativeMode::DERIVATIVE_ON_MEASUREMENT;
  z_controller.derivativeMode = RSLA::DerivativeMode::DERIVATIVE_ON_MEASUREMENT;
  
  roll_controller.derivativeMode = RSLA::DerivativeMode::DERIVATIVE_ON_MEASUREMENT;
  pitch_controller.derivativeMode = RSLA::DerivativeMode::DERIVATIVE_ON_MEASUREMENT;
  yaw_controller.derivativeMode = RSLA::DerivativeMode::DERIVATIVE_ON_MEASUREMENT;

  z_controller.kP = 30.0;
  z_controller.kI = 1.0;
  z_controller.kD = 10.0;
  z_controller.antiwindup = 2.0;
  z_controller.enableAntiwindup = true;
  z_controller.bias = 7.0;
  z_controller.constraint = 15.0;
  z_controller.enableConstraint = true;

  roll_controller.kP = 0.1;
  roll_controller.kD = 0.025;
  roll_controller.constraint = 5;
  roll_controller.enableConstraint = true;

  pitch_controller.kP = 0.1;
  pitch_controller.kD = 0.025;
  pitch_controller.bias = 0.1;
  pitch_controller.constraint = 5;
  pitch_controller.enableConstraint = true;

  yaw_controller.kP = 0.2;
  yaw_controller.kI = 0.025;
  yaw_controller.kD = 0.0;
  yaw_controller.antiwindup = 0.25;
  yaw_controller.enableAntiwindup = true;
  yaw_controller.bias = 0.0;
  yaw_controller.constraint = 15.0;
  yaw_controller.enableConstraint = true;
}

void initialize_message_data() {
  status_msg.data = 0;
  orientation_msg.orientation.yaw = 0;
  orientation_msg.orientation.pitch = 0;
  orientation_msg.orientation.roll = 0;
  orientation_msg.position.x = 0;
  orientation_msg.position.y = 0;
  orientation_msg.position.z = 0;
}

void get_imu() {
      bno_euler_data = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      yaw = bno_euler_data.x();
      pitch = bno_euler_data.y();
      roll = -bno_euler_data.z();
      bno.getCalibration(&bno_sys_cal, &bno_gyro_cal, &bno_acc_cal, &bno_mag_cal);
}

void get_baro() {
  baro.read();
  baro_depth_data = baro.depth();
  z = baro_depth_data - surface_offset;
}

void get_battery() {
#ifdef DEBUG
  battery_voltage = 16.8;
#else
  battery_voltage = ADS.getValue() * (16.23 / 26796);
#endif
}

void update_pids(float dt) {
  yaw_controller.update(yaw, dt);
  pitch_controller.update(pitch, dt);
  roll_controller.update(roll, dt);
  x_controller.update(x, dt);
  y_controller.update(y, dt);
  z_controller.update(z, dt);
}

void update_control_vector() {
  if (x_mode == ControllerMode::EFFORT) {
    control_vector[0] = x_effort;
  }
  else {
    control_vector[0] = x_controller.output;
  }

  if (y_mode == ControllerMode::EFFORT) {
    control_vector[1] = y_effort;
  }
  else {
    control_vector[1] = y_controller.output;
  }

  if (z_mode == ControllerMode::EFFORT) {
    control_vector[2] = z_effort;
  }
  else {
    control_vector[2] = z_controller.output;
  }

  if (roll_mode == ControllerMode::EFFORT) {
    control_vector[3] = roll_effort;
  }
  else {
    control_vector[3] = roll_controller.output;
  }

  if (pitch_mode == ControllerMode::EFFORT) {
    control_vector[4] = pitch_effort;
  }
  else {
    control_vector[4] = pitch_controller.output;
  }

  if (yaw_mode == ControllerMode::EFFORT) {
    control_vector[5] = yaw_effort;
  }
  else {
    control_vector[5] = yaw_controller.output;
  }
}

void update_output_vectors() {
  if(vehicle_armed)
    {
      for(int i = 0; i < 8; i++)
      {
        thruster_forces[i] = thruster_solver.getForce(i);
        thruster_pwm[i] = thruster_solver.getPWM(i, battery_voltage);
      }
    }
    else
    {
      for(int i = 0; i < 8; i++)
      {
        thruster_forces[i] = thruster_solver.getForce(i);
        thruster_pwm[i] = 1500; // Output idle PWM when disarmed
      }
    }
}

void control_ball_dropper() {
    switch (ball_dropper_msg.data) {
        case 0: ball_dropper.write(90); break; // neutral
        case 1: ball_dropper.write(135); break; // left ball
        case 2: ball_dropper.write(45); break; // right ball
        default: break;
    }
}

void output_to_pwm(uint32_t loop_time_millis) {
  if(loop_time_millis > last_pwm_write_ms + pwm_write_interval_ms)
    {
      last_pwm_write_ms = loop_time_millis;

      for(int i = 0; i < 8; i++)
      {
        pwm_driver.writeMicroseconds(7 - i, thruster_pwm[i]);
      }
    }
}
