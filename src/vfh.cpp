/* <Copyright <sshyun0909> */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/impl/utils.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <memory>
#include <limits>

class VFH : public rclcpp::Node
{
public:
  VFH()
  : Node("VFH"),
    scan_updated_(false), robot_reached_(false), robot_aligned_(false), sub_goal_reached_(true)
  {
    this->declare_parameter<std::string>("global_frame", "map");
    this->declare_parameter<std::string>("robot_frame", "base_footprint");
    this->declare_parameter<float>("robot_bubble", 30.0);
    this->declare_parameter<double>("max_velocity", 0.4);
    this->declare_parameter<double>("min_velocity", 0.0);
    this->declare_parameter<double>("max_yawrate", 0.25);
    this->declare_parameter<double>("dist_goal_th", 0.2);
    this->declare_parameter<double>("angle_goal_th", 0.02);
    this->declare_parameter<double>("obstacle_th", 1.25);
    this->declare_parameter<double>("emergency_th", 0.6);
    this->declare_parameter<double>("sub_goal_th", 1.5);
    this->declare_parameter<double>("safety_distance", 2.0);
    this->declare_parameter<double>("goal_point_x", 0.0);
    this->declare_parameter<double>("goal_point_y", 0.0);
    this->declare_parameter<double>("goal_point_yaw", 0.0);

    this->get_parameter("global_frame", goal_.header.frame_id);
    this->get_parameter("robot_frame", robot_frame_);
    this->get_parameter("robot_bubble", robot_bubble_);
    this->get_parameter("max_velocity", max_velocity_);
    this->get_parameter("min_velocity", min_velocity_);
    this->get_parameter("max_yawrate", max_yawrate_);
    this->get_parameter("dist_goal_th", dist_goal_th_);
    this->get_parameter("angle_goal_th", angle_goal_th_);
    this->get_parameter("obstacle_th", obstacle_th_);
    this->get_parameter("emergency_th", emergency_th_);
    this->get_parameter("sub_goal_th", sub_goal_th_);
    this->get_parameter("safety_distance", safety_distance_);
    this->get_parameter("goal_point_x", goal_.pose.position.x);
    this->get_parameter("goal_point_y", goal_.pose.position.y);
    this->get_parameter("goal_point_yaw", goal_yaw_);

    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw_);
    goal_.pose.orientation = tf2::toMsg(q);

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&VFH::scan_callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void process()
  {
    rclcpp::Rate loop_rate(20);
    while (rclcpp::ok()) {
      geometry_msgs::msg::Twist cmd_vel;

      if (scan_updated_) {
        if (!calculate_pose(robot_transform_)) {
          RCLCPP_WARN(this->get_logger(), "Failed to transform");
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
          velocity_pub_->publish(cmd_vel);
          loop_rate.sleep();
          continue;
        }

        if (obstacle_detection()) {
          if (sub_goal_reached_) {
            create_histogram();
          }
        }

        if (!emergency_break()) {
          if (!sub_goal_reached_) {
            cmd_vel = vfh_velocity();
          } else {
            cmd_vel = move_robot();
          }
        } else {
          cmd_vel.linear.x = min_velocity_;
          cmd_vel.angular.z = 0.0;
        }
        velocity_pub_->publish(cmd_vel);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to update scan data");
      }
      rclcpp::spin_some(this->shared_from_this());
      loop_rate.sleep();
    }
  }

  bool calculate_pose(tf2::Transform & robot_transform_)
  {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform(
        goal_.header.frame_id, robot_frame_, tf2::TimePointZero);
      tf2::fromMsg(transform_stamped.transform, robot_transform_);

      robot_x_ = robot_transform_.getOrigin().x();
      robot_y_ = robot_transform_.getOrigin().y();
      robot_yaw_ = tf2::impl::getYaw(robot_transform_.getRotation());

      double goal_x = goal_.pose.position.x;
      double goal_y = goal_.pose.position.y;

      double diff_x = goal_x - robot_x_;
      double diff_y = goal_y - robot_y_;

      target_yaw_ = atan2(diff_y, diff_x);
      yaw_error_ = target_yaw_ - robot_yaw_;

      while (yaw_error_ > M_PI) {yaw_error_ -= 2.0 * M_PI;}
      while (yaw_error_ < -M_PI) {yaw_error_ += 2.0 * M_PI;}

      return true;
    } catch (tf2::TransformException & ex) {
      return false;
    }
  }

  bool obstacle_detection()
  {
    geometry_msgs::msg::Twist cmd_vel;
    for (size_t i = 0; i < scan_msg_->ranges.size(); ++i) {
      double angle = scan_msg_->angle_min + i * scan_msg_->angle_increment;
      double angle_deg = angle * 180.0 / M_PI;

      if ((angle_deg >= -180 && angle_deg <= -90) || (angle_deg >= 90 && angle_deg <= 180)) {
        if (scan_msg_->ranges[i] < obstacle_th_) {
          return true;
        }
      }
    }
    return false;
  }

  bool emergency_break()
  {
    for (size_t i = 0; i < scan_msg_->ranges.size(); ++i) {
      if (scan_msg_->ranges[i] < emergency_th_) {
        RCLCPP_WARN(this->get_logger(), "Emergency stop! Obstacle detected within threshold.");
        return true;
      }
    }
    return false;
  }

  geometry_msgs::msg::Twist move_robot()
  {
    geometry_msgs::msg::Twist cmd_vel;
    double distance_to_goal = std::hypot(
      goal_.pose.position.x - robot_transform_.getOrigin().x(),
      goal_.pose.position.y - robot_transform_.getOrigin().y());
    if (dist_goal_th_ > distance_to_goal) {
      robot_reached_ = true;
    }

    if (!robot_aligned_ && robot_reached_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = yaw_error_;
      RCLCPP_INFO(
        this->get_logger(), "Align velocity: speed: %.3f, angular: %.3f", cmd_vel.linear.x,
        cmd_vel.angular.z);
      if (std::abs(yaw_error_) < angle_goal_th_) {
        robot_aligned_ = true;
      }
    } else if (robot_aligned_ && robot_reached_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "VFH Navigation Successed");
    } else {
      cmd_vel.linear.x = max_velocity_;
      cmd_vel.angular.z = std::min(std::max(yaw_error_, -max_yawrate_), max_yawrate_);
      RCLCPP_INFO(
        this->get_logger(), "Move velocity: speed: %.3f, angular: %.3f", cmd_vel.linear.x,
        cmd_vel.angular.z);
    }
    return cmd_vel;
  }

  geometry_msgs::msg::Twist vfh_velocity()
  {
    geometry_msgs::msg::Twist cmd_vel;
    sub_yaw_error_ = update_sub_pose();

    if (std::abs(sub_yaw_error_) > angle_goal_th_) {
      if (best_index_ == left_index_) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = std::min(sub_yaw_error_, max_yawrate_);
        RCLCPP_INFO(
          this->get_logger(), "Align Left VFH velocity: speed: %.3f, angular: %.3f",
          cmd_vel.linear.x, cmd_vel.angular.z);
      } else if (best_index_ == right_index_) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = std::max(sub_yaw_error_, -max_yawrate_);
        RCLCPP_INFO(
          this->get_logger(), "Align Right VFH velocity: speed: %.3f, angular: %.3f",
          cmd_vel.linear.x, cmd_vel.angular.z);
      }
    } else {
      cmd_vel.linear.x = std::min(sub_goal_distance_, max_velocity_);
      cmd_vel.angular.z = sub_yaw_error_;
      RCLCPP_INFO(
        this->get_logger(), "Move VFH velocity: speed: %.3f, angular: %.3f", cmd_vel.linear.x,
        cmd_vel.angular.z);
      if (sub_goal_distance_ < dist_goal_th_) {
        cmd_vel.linear.x = min_velocity_;
        cmd_vel.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Sub Goal Point Arrived!");
        sub_goal_reached_ = true;
      }
    }
    return cmd_vel;
  }

  void create_histogram()
  {
    std::vector<float> distance_vector(360, std::numeric_limits<float>::infinity());
    for (size_t i = 0; i < scan_msg_->ranges.size(); ++i) {
      if (scan_msg_->ranges[i] < scan_msg_->range_min) {
        scan_msg_->ranges[i] = scan_msg_->range_min;
      } else if (scan_msg_->ranges[i] > 10.0) {
        scan_msg_->ranges[i] = 10;
      }

      double angle = scan_msg_->angle_min + i * scan_msg_->angle_increment;
      int vector_index = static_cast<int>(std::round((angle * 180 / M_PI) + 180));
      distance_vector[vector_index] = std::min(distance_vector[vector_index], scan_msg_->ranges[i]);
    }

    for (std::vector<float>::size_type i = 0; i < distance_vector.size() / 2; ++i) {
      if (distance_vector[i] > safety_distance_) {
        left_index_ = i + robot_bubble_;
        break;
      }
    }
    for (std::vector<float>::size_type i = 0; i < distance_vector.size() / 2; ++i) {
      if (distance_vector[distance_vector.size() - i - 1] > safety_distance_) {
        right_index_ = i + robot_bubble_;
        break;
      }
    }
    RCLCPP_INFO(this->get_logger(), "left index : %d, right_index : %d", left_index_, right_index_);
    best_index_ =
      (left_index_ <= right_index_) ? left_index_ : right_index_;
    RCLCPP_INFO(
      this->get_logger(), "Selected direction: %s, Index: %d",
      (best_index_ == left_index_) ? "Left" : "Right", best_index_);

    sub_goal_angle_ = best_index_ * M_PI / 180.0;
    if (best_index_ == left_index_) {
      if (robot_yaw_ + sub_goal_angle_ <= M_PI / 2) {
        sub_goal_x_ = robot_x_ - sub_goal_th_ * cos(robot_yaw_ + sub_goal_angle_);
        sub_goal_y_ = robot_y_ + sub_goal_th_ * sin(robot_yaw_ + sub_goal_angle_);
      } else if (robot_yaw_ + sub_goal_angle_ > M_PI / 2) {
        sub_goal_x_ = robot_x_ - sub_goal_th_ * cos(M_PI - (robot_yaw_ + sub_goal_angle_));
        sub_goal_y_ = robot_y_ + sub_goal_th_ * sin(M_PI - (robot_yaw_ + sub_goal_angle_));
      }
    } else if (best_index_ == right_index_) {
      if (robot_yaw_ + sub_goal_angle_ <= M_PI / 2) {
        sub_goal_x_ = robot_x_ + sub_goal_th_ * cos(robot_yaw_ - sub_goal_angle_);
        sub_goal_y_ = robot_y_ + sub_goal_th_ * sin(robot_yaw_ - sub_goal_angle_);
      } else if (robot_yaw_ + sub_goal_angle_ > M_PI / 2) {
        sub_goal_x_ = robot_x_ - sub_goal_th_ * cos(M_PI - (robot_yaw_ - sub_goal_angle_));
        sub_goal_y_ = robot_y_ + sub_goal_th_ * sin(M_PI - (robot_yaw_ - sub_goal_angle_));
      }
    }

    sub_goal_.pose.position.x = sub_goal_x_;
    sub_goal_.pose.position.y = sub_goal_y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, robot_yaw_ + sub_goal_angle_);
    sub_goal_.pose.orientation = tf2::toMsg(q);
    RCLCPP_INFO(
      this->get_logger(), "Update Sub Goal: x=%.3f, y=%.3f, yaw=%.3f", sub_goal_x_, sub_goal_y_,
      sub_goal_.pose.orientation.w);

    if (best_index_ == left_index_) {
      sub_target_yaw_ = robot_yaw_ + sub_goal_angle_;
    } else if (best_index_ == right_index_) {
      sub_target_yaw_ = robot_yaw_ - sub_goal_angle_;
    }
    RCLCPP_INFO(
      this->get_logger(), "sub_target_yaw: %.3f", sub_target_yaw_);

    sub_goal_reached_ = false;
  }

  double update_sub_pose()
  {
    double sub_diff_x = sub_goal_x_ - robot_x_;
    double sub_diff_y = sub_goal_y_ - robot_y_;
    sub_goal_distance_ = std::hypot(sub_diff_x, sub_diff_y);

    sub_yaw_error_ = sub_target_yaw_ - robot_yaw_;

    RCLCPP_INFO(
      this->get_logger(), "robot_x: %.3f, robot_y: %.3f, robot_yaw: %.3f",
      robot_x_, robot_y_, robot_yaw_);
    RCLCPP_INFO(
      this->get_logger(), "sub_goal_distance: %.3f, sub_yaw_error: %.3f",
      sub_goal_distance_, sub_yaw_error_);
    return sub_yaw_error_;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_msg_ = msg;
    scan_updated_ = true;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::PoseStamped sub_goal_;
  tf2::Transform robot_transform_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;

  std::string robot_frame_;
  bool scan_updated_, robot_reached_, robot_aligned_, sub_goal_reached_;
  double max_velocity_, min_velocity_, max_yawrate_, goal_yaw_;
  double robot_x_, robot_y_, robot_yaw_;
  double target_yaw_, yaw_error_, sub_target_angle_, sub_yaw_error_, sub_target_yaw_;
  double dist_goal_th_, angle_goal_th_, emergency_th_, obstacle_th_;
  double sub_goal_angle_, sub_goal_x_, sub_goal_y_, sub_goal_distance_, sub_goal_th_,
    safety_distance_;
  int best_index_, left_index_, right_index_;
  float robot_bubble_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VFH>();
  node->process();
  rclcpp::shutdown();
  return 0;
}
