#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#define PI 3.14159265359

double L = 0.35; // distance between axes
double R = 0.1016; // wheel radius

double encoder_left = 0;
double encoder_right = 0;
double encoder_dt = 0;

double acc_x = 0.0;
double acc_y = 0.0;
double acc_z = 0.0;

double gyro_vx = 0.0;
double gyro_vy = 0.0;
double gyro_vz = 0.0;

double v_encoder = 0;
double w_encoder = 0;

ros::Time encoder_time;
bool init = false;

void handle_encoder(const geometry_msgs::Vector3Stamped& encoder) {
  encoder_left = encoder.vector.x;
  encoder_right = encoder.vector.y;
  encoder_dt = encoder.vector.z;

  //ROS_INFO("encoder_left %lf - encoder_right %lf", encoder.vector.x, encoder.vector.y);
}

// Gyro Function from Smartphone
void handle_gyro( const sensor_msgs::Imu::ConstPtr& msg) {
  gyro_vx = msg->angular_velocity.x;
  gyro_vy = msg->angular_velocity.y;
  gyro_vz = msg->angular_velocity.z;

  // gyro_vz = (msg->angular_velocity.z*PI)/180; // rad/s

  acc_x = msg->linear_acceleration.x;
  acc_y = msg->linear_acceleration.y;
  acc_z = msg->linear_acceleration.z;
}

// Robot Differential Drive Reverse Kinematic
void reverse_kinematics(){
  v_encoder = (encoder_left + encoder_right)/2; // linear
  w_encoder = (encoder_right - encoder_left)/L; // angular

  //ROS_INFO("reverse_kinematics - v_encoder %lf - dth_encoder %lf", v_encoder, dth_encoder);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_controller");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber enconder_sub = nh.subscribe("/data/enconder", 100, handle_encoder);
  ros::Subscriber gyro_sub = nh.subscribe("/mpu6050_imu/data", 100, handle_gyro);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom_imu_encoder", 50);

  // Crete tf - base link and Odometry
  tf::TransformBroadcaster baselink_broadcaster;
  tf::TransformBroadcaster odom_broadcaster;

  double alpha = 0.0;
  bool use_imu = false;
  double vx = 0.0;
  double vth = 0.0;
  double dth = 0.0;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vz = 0.0;

  nh_private_.getParam("alpha", alpha);
  nh_private_.getParam("use_imu", use_imu);

  ros::Time current_time, last_time;
  ros::Rate r(10.0);

  while(nh.ok()){

    ros::spinOnce(); //check for incoming messages

    if(!init){

      reverse_kinematics();
      last_time = ros::Time::now();
      init = true;

    }else if(init){

      reverse_kinematics(); // return v_encoder and w_encoder
      current_time = ros::Time::now();
      //compute odometry in a typical way given the velocities of the robot
      double dt = (current_time - last_time).toSec();

      // double vx = acc_x * dt;
      // double vy = acc_y * dt;

      // // delta position and delta orientation
      // double delta_x = vx * dt;
      // double delta_y = vy * dt;
      // double delta_th = gyro_vz * dt;

      if (use_imu) {
        vz = alpha*w_encoder + (1-alpha)*gyro_vz;
      } 
      else {
        vz = w_encoder;
      }

      double vx = v_encoder * cos(th);
      double vy = v_encoder * sin(th);

      double delta_x = vx * dt;
      double delta_y = vy * dt;
      double delta_th = vz * dt;

      // position and orientation
      x += delta_x;
      y += delta_y;
      th += delta_th;

      // ROS_INFO("DEBUG - th: %lf - dth: %lf - dt: %lf", th, delta_th, dt);

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "imu_link";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "imu_link";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.linear.z = 0;
      odom.twist.twist.angular.x = 0;
      odom.twist.twist.angular.y = 0;
      odom.twist.twist.angular.z = vth;

      //set the covariance encoder
      odom.pose.covariance[0] = 5.0;
      odom.pose.covariance[7] = 5.0;
      odom.pose.covariance[14] = 1e-3;
      odom.pose.covariance[21] = 0.1;
      odom.pose.covariance[28] = 0.1;
      odom.pose.covariance[35] = 0.1;
      odom.twist.covariance[0] = 1.0;
      odom.twist.covariance[7] = 1e6;
      odom.twist.covariance[14] = 1e6;
      odom.twist.covariance[21] = 1e6;
      odom.twist.covariance[28] = 1e6;
      odom.twist.covariance[35] = 0.5;

      //publish the message
      odom_pub.publish(odom);
    }
      // update the time
    last_time = current_time;
    r.sleep();
  }
}
