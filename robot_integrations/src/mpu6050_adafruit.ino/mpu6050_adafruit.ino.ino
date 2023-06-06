// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <ros.h>
#include <sensor_msgs/Imu.h>

Adafruit_MPU6050 mpu;

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;

ros::Publisher imu_pub("/mpu6050_imu/data", &imu_msg);

#define LED_PIN 2
#define DEBUG 1
bool blinkState = false;


float scale = 0.01;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float gravity [3] = {0,0,0};

void setup(void) {

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);

  // initialize node ros
  nh.initNode();
  nh.advertise(imu_pub);

  // while (!Serial) {
  //   delay(10); // will pause Zero, Leonardo, etc until serial console opens
  // }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_5_HZ);
  // Serial.println("");
  pinMode(LED_PIN, OUTPUT);

  // Serial.println("");
  // Serial.println("Calibrating...");
  
  delay(500);

  // /* Get new sensor events with the readings */
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);

  // gravity[0] = a.acceleration.x;
  // gravity[1] = a.acceleration.y;
  // gravity[2] = a.acceleration.z;
  // gx_offset += g.gyro.x;
  // gy_offset += g.gyro.y;
  // gz_offset += g.gyro.z;

  //   /* Print out the values */
  // Serial.print("g-AccelX:");
  // Serial.print(gravity[0]);
  // Serial.print(",");
  // Serial.print("g-AccelY:");
  // Serial.print(gravity[1]);
  // Serial.print(",");
  // Serial.print("g-AccelZ:");
  // Serial.print(gravity[2]);
  // Serial.print(", ");
  // Serial.print("gx_offset:");
  // Serial.print(gx_offset);
  // Serial.print(",");
  // Serial.print("gy_offset:");
  // Serial.print(gy_offset);
  // Serial.print(",");
  // Serial.print("gz_offset:");
  // Serial.print(gz_offset);
  // Serial.println("");
 
  // delay(500);

  // Serial.println(" Ready to start reading...");
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // /* Print out the values */
  // Serial.print("AccelX:");
  // Serial.print(a.acceleration.x);
  // Serial.print(",");
  // Serial.print("AccelY:");
  // Serial.print(a.acceleration.y);
  // Serial.print(",");
  // Serial.print("AccelZ:");
  // Serial.print(a.acceleration.z);
  // Serial.print(", ");
  // Serial.print("GyroX:");
  // Serial.print(g.gyro.x);
  // Serial.print(",");
  // Serial.print("GyroY:");
  // Serial.print(g.gyro.y);
  // Serial.print(",");
  // Serial.print("GyroZ:");
  // Serial.print(g.gyro.z);
  // Serial.println("");

  imu_msg.angular_velocity.x = g.gyro.x + 0.12;
  imu_msg.angular_velocity.y = g.gyro.y + 0.01;
  imu_msg.angular_velocity.z = g.gyro.z + 0.01;

  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;

  // if (DEBUG)
  // {
  //   /* Print out the values */
  //   Serial.print("AccelX:");
  //   Serial.print(a.acceleration.x);
  //   Serial.print(",");
  //   Serial.print("AccelY:");
  //   Serial.print(a.acceleration.y);
  //   Serial.print(",");
  //   Serial.print("AccelZ:");
  //   Serial.print(a.acceleration.z);
  //   Serial.print(", ");
  //   Serial.print("GyroX:");
  //   Serial.print(imu_msg.angular_velocity.x);
  //   Serial.print(",");
  //   Serial.print("GyroY:");
  //   Serial.print(imu_msg.angular_velocity.y);
  //   Serial.print(",");
  //   Serial.print("GyroZ:");
  //   Serial.print(imu_msg.angular_velocity.z);
  //   Serial.println("");
  // }


  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  imu_pub.publish(&imu_msg);

  nh.spinOnce();

  delay(10);
}