#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
// #include <std_msgs/msg/float32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
timespec ts;

#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gg;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


rcl_publisher_t publisher;
// rcl_publisher_t publisher1;
sensor_msgs__msg__Imu msg;
// std_msgs__msg__Float32 msg1;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     msg.data++;
//   }
// }

void setup() {
  // Configure serial transport
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  Serial.begin(115200);
  extern int clock_gettime(clockid_t unused, struct timespec *tp);
  mpu.initialize();
    devStatus = mpu.dmpInitialize();
      mpu.setXAccelOffset(-102);
  mpu.setYAccelOffset(-3228);
  mpu.setZAccelOffset(1073);
  mpu.setXGyroOffset(408);
  mpu.setYGyroOffset(5);
  mpu.setZGyroOffset(-6);// 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_imu", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "micro_ros_imu"));

  // RCCHECK(rclc_publisher_init_default(
  //   &publisher1,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  //   "roll"));

  // create timer,
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // msg.data = 0;
}

void loop() {
  // delay(100);
  if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

      clock_gettime(CLOCK_REALTIME, &ts);
      msg.header.stamp.nanosec = ts.tv_nsec;
      msg.header.stamp.sec = ts.tv_sec;
        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        // Serial.print("quat\t");
        // Serial.print(q.w);
        // Serial.print("\t");
        // Serial.print(q.x);
        // Serial.print("\t");
        // Serial.print(q.y);
        // Serial.print("\t");
        // Serial.println(q.z);
        msg.orientation.w = q.w;
        msg.orientation.x = q.x;
        msg.orientation.y = q.y;
        msg.orientation.z = q.z;

        // mpu.dmpGetGravity(&gravity, &q);

        // // display initial world-frame acceleration, adjusted to remove gravity
        // // and rotated based on known orientation from quaternion
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
        msg.linear_acceleration.x = aaWorld.x;
        msg.linear_acceleration.y = aaWorld.y;
        msg.linear_acceleration.z = aaWorld.z;
        // Serial.print("aworld\t");
        // Serial.print(aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);
        // Serial.print("\t");
        // Serial.print(aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);
        // Serial.print("\t");
        // Serial.println(aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);

        // // display initial world-frame acceleration, adjusted to remove gravity
        // // and rotated based on known orientation from quaternion
        mpu.dmpGetGyro(&gg, fifoBuffer);
        mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
        msg.angular_velocity.x = ggWorld.x;
        msg.angular_velocity.y = ggWorld.y;
        msg.angular_velocity.z = ggWorld.z;
        // Serial.print("ggWorld\t");
        // Serial.print(ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD);
        // Serial.print("\t");
        // Serial.print(ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD);
        // Serial.print("\t");
        // Serial.println(ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD);

        // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // msg1.data = ypr[2];
        // // Serial.print("ypr\t");
        // Serial.print(ypr[0] );
        // Serial.print(",");
        // Serial.print(ypr[1] );
        // Serial.print(",");
        // Serial.println(ypr[2] );
        // msg.header.frame_id.data = "imu_link";
        
        
        // Serial.println();

        // delay(50);
    }
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    // RCSOFTCHECK(rcl_publish(&publisher1, &msg1, NULL));
    
  // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}