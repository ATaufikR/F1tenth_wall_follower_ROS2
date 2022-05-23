// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#include <string.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
// #include "std_msgs/msg/string.hpp"

#define LINEAR_VELOCITY  0.03
#define SLOW_LINEAR_VELOCITY  0.01
#define ANGULAR_VELOCITY 1.82

using namespace std;

class Turtlebot3Drive : public rclcpp::Node
{
public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Variables

  float left_side;
  float right_side;
  float front_left_side;
  float front_right_side;
  float range_min; 
  float range_max;

  int count_ = 0;

  float theta;    

  float min_val_;
  float max_val_;
  float kp_;
  float ki_;
  float kd_;
  double integral_;
  double derivative_;
  double prev_error_;
  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  int front;
  int left1;
  int left2;
  int left3;
  int left4;
  int right1;
  int right2;
  int right3;
  int right4;

  int countr;
  int counts;
  float sum;
  float sum1;
  float range;
  float range1;
  float d;
  float d1;

  float diff;
  int ang_max;
  int ang_min;

  float degree1;
  float degree2;
  float alfa;
  float last_sum;
  float last_countr;
  float last_sum1;
  float last_counts;
  
  float front_view;
  // Function prototypes
  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  // void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);


  // double compute(float setpoint, float measured_value);

  void full_stop();
  void find_wall();
  void turn_left(float ang);
  void turn_right(float ang);
  void follow_wall(float ang);
  void follow_wall_slow(float ang);
  void move_backward();
};