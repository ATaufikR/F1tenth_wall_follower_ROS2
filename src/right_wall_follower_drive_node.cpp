#include <cstdio>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
// #include "std_msgs/msg/string.hpp"

#define LINEAR_VELOCITY 0.03
#define SLOW_LINEAR_VELOCITY 0.02
#define SLOWER_LINEAR_VELOCITY 0.01
#define ANGULAR_VELOCITY 1.82

#include <memory>

#include <math.h>

#define RAD2DEG(x) ((x)*180. / M_PI)
#define DEG2RAD(x) ((x)*M_PI / 180.)

using namespace std::chrono_literals;

class right_wall_follower : public rclcpp::Node
{
public:
  right_wall_follower()
      : Node("right_wall_follower_drive_node")
  {

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Initialise publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // Initialize subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(
            &right_wall_follower::scan_callback,
            this,
            std::placeholders::_1));
    /************************************************************
    ** Initialise ROS timers
    ************************************************************/
    update_timer_ = this->create_wall_timer(10ms, std::bind(&right_wall_follower::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "Wall follower right drive node has been initialized");
  }

private:
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

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    front = 0;
    left1 = -30;
    right2 = -60;
    right3 = -90;
    right4 = -120;
    left1 = 30;
    left2 = 60;
    left3 = 90;
    left4 = 120;
    countr = 0;
    counts = 0;
    sum = 0;
    sum1 = 0;
    range = 0;
    range1 = 0;
    diff = 1;
    ang_max = 180;
    ang_min = -180;

    degree1 = -90.0;
    degree2 = -30.0;
    // alfa = 0.0;

    std::vector<float> laser_ranges;
    laser_ranges = msg->ranges;

    left_side = 0.0;
    right_side = 0.0;
    range_max = msg->range_max;
    range_min = msg->range_min;

    int count = msg->scan_time / msg->time_increment;

    for (int i = 0; i < count; i++)
    {

      float degree = RAD2DEG(msg->angle_min + msg->angle_increment * i);

      if (degree > -1.0 && degree < 1.0)
      {
        if (laser_ranges[i] > 0)
        {
          front_view = laser_ranges[i];
        }
        else
        {
          front_view = range_max;
        }
      }

      if (degree < (degree1 + diff) && degree > (degree1 - diff))
      {

        // printf("%f, %f\n", msg->ranges[i], msg->ranges[i-1]);
        if (laser_ranges[i] > 0)
        {
          sum = sum + msg->ranges[i];
          countr++;
          last_sum = sum;
          last_countr = countr;
          // printf("%f\n", sum);
        }
        else
        {
          sum = last_sum;
          countr = last_countr;
          // printf("%f\n", msg->ranges[i]);
          // printf("no wall detected\n");
        }
      }

      if (degree < (degree2 + diff) && degree > (degree2 - diff))
      {
        if (laser_ranges[i] > 0)
        {
          sum1 = sum1 + msg->ranges[i];
          counts++;
          last_sum1 = sum1;
          last_counts = counts;
        }
        else
        {
          sum1 = last_sum1;
          counts = last_counts;
          // printf("%f\n", msg->ranges[i]);
        }
      }
    }

    range = sum / countr;
    range1 = sum1 / counts;
    right_side = range;
    front_right_side = range1;

    theta = degree2 - degree1;

    float a = range1 * cos(DEG2RAD(theta));
    float b = range1 * sin(DEG2RAD(theta));
    alfa = RAD2DEG(atan((a - range) / (b)));

    d = range * cos(DEG2RAD(alfa)); // instance distance between center of the lidar with right wall

    float c = 0.3; // distance for prediction

    d1 = d + c * sin(DEG2RAD(alfa));
  }

  // Function prototypes
  void update_callback()
  {
    float wall_distance = 0.3;
    float front_dist = 2.0;

    double error;
    double pid;
    min_val_ = -1.82;
    max_val_ = 1.82;
    kp_ = 0.6;
    ki_ = 0.0;
    kd_ = 20.0;

    // setpoint is constrained between min and max to prevent pid from having too much error
    // if (d1 > 0)
    // {
    error = wall_distance - d1;
    integral_ += error;
    derivative_ = error - prev_error_;

    if (wall_distance == 0 && error == 0)
    {
      integral_ = 0;
    }

    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    if (pid > max_val_)
    {
      pid = max_val_;
    }

    if (pid < min_val_)
    {
      pid = min_val_;
    }
    // }
    if (front_view <= front_dist)
    {
      // full_stop();
      turn_left(1.82);
    }

    else
    {
      if (pid > 1.0 || pid < -1.0)
      {
        follow_wall_slow(pid);
      }

      else if (pid > 1.81 || pid < -1.81)
      {
        follow_wall_slower(pid);
      }

      else
      {
        follow_wall(pid);
      }
    }
    // printf("%f, %f, %f, %f,heading angle: %f,instance distance: %f,future distance: %f,pid value: %f, front view: %f\n", degree1, range, degree2, range1, alfa, d, d1, pid, front_view);
    // printf("%f,%f,%f,%f\n", front_view, d1, wall_distance, pid);
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "%f,%f,%f,%f,%f", front_view, d1, wall_distance, pid, alfa);
    // publisher_->publish(message);
  }

  void update_cmd_vel(double linear, double angular)
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    cmd_vel_pub_->publish(cmd_vel);
  }

  void full_stop()
  {
    update_cmd_vel(0.0, 0.0);
  }
  void find_wall()
  {
    update_cmd_vel(LINEAR_VELOCITY, 0.0);
  }
  void turn_left(float ang)
  {
    update_cmd_vel(SLOWER_LINEAR_VELOCITY, ang);
  }
  void turn_right(float ang)
  {
    update_cmd_vel(SLOW_LINEAR_VELOCITY, -1 * ang);
  }
  void follow_wall(float ang)
  {
    update_cmd_vel(LINEAR_VELOCITY, ang);
  }
  void follow_wall_slow(float ang)
  {
    update_cmd_vel(SLOW_LINEAR_VELOCITY, ang);
  }
  void follow_wall_slower(float ang)
  {
    update_cmd_vel(SLOWER_LINEAR_VELOCITY, ang);
  }
  void move_backward()
  {
    update_cmd_vel(-1.0 * LINEAR_VELOCITY, 0.0);
  }

  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<right_wall_follower>());
  rclcpp::shutdown();

  return 0;
}
