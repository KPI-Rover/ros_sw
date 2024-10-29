#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

class MoveAndTurnNode : public rclcpp::Node
{
public:
    MoveAndTurnNode()
        : Node("move_and_turn_node"), distance_moved_(0.0), angle_turned_(0.0),
          phase_(0), initial_x_(0.0), initial_y_(0.0), initial_yaw_(0.0)
    {
        // Publisher to send velocity commands with TwistStamped
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);

        // Subscriber to odometry data
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diff_drive_controller/odom", 10, std::bind(&MoveAndTurnNode::odomCallback, this, std::placeholders::_1));

        // Subscriber to IMU data
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&MoveAndTurnNode::imuCallback, this, std::placeholders::_1));

        // Timer to control the robot
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MoveAndTurnNode::controlLoop, this));
    }

private:
    // Odometry callback to track position
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Calculate the distance moved
        distance_moved_ = std::sqrt(std::pow(current_x_ - initial_x_, 2) +
                                    std::pow(current_y_ - initial_y_, 2));
    }

    // IMU callback to track orientation and angular velocity
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract orientation from IMU data (as quaternion)
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        // Convert quaternion to roll, pitch, and yaw
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);

        // Calculate the angle turned
        angle_turned_ = current_yaw_ - initial_yaw_;
    }

    void controlLoop()
    {
        geometry_msgs::msg::TwistStamped cmd_msg;
        cmd_msg.header.stamp = this->get_clock()->now();
        cmd_msg.header.frame_id = "base_link";  // Assuming the base frame is "base_link"

        // Phase 0: Move forward a certain distance
        if (phase_ == 0)
        {
            if (distance_moved_ < target_distance_)
            {
                cmd_msg.twist.linear.x = 0.2;  // Move forward
            }
            else
            {
                cmd_msg.twist.linear.x = 0.0;  // Stop moving forward
                phase_ = 1;                    // Transition to turning phase
                initial_yaw_ = current_yaw_;    // Reset initial yaw for turning
            }
        }
        // Phase 1: Turn by a certain angle using IMU data
        else if (phase_ == 1)
        {
            if (std::fabs(angle_turned_) < target_angle_)
            {
                cmd_msg.twist.angular.z = 0.5;  // Turn
            }
            else
            {
                cmd_msg.twist.angular.z = 0.0;  // Stop turning
                phase_ = 2;                     // Transition to next move phase
                initial_x_ = current_x_;         // Reset initial position
                initial_y_ = current_y_;
            }
        }
        // Phase 2: Move forward again
        else if (phase_ == 2)
        {
            if (distance_moved_ < target_distance_)
            {
                cmd_msg.twist.linear.x = 0.2;  // Move forward again
            }
            else
            {
                cmd_msg.twist.linear.x = 0.0;  // Stop
                phase_ = 3;                    // Stop after completing the motion
                RCLCPP_INFO(this->get_logger(), "Movement completed!");
            }
        }

        // Publish the velocity command
        cmd_vel_pub_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double current_x_, current_y_, current_yaw_;
    double initial_x_, initial_y_, initial_yaw_;
    double distance_moved_, angle_turned_;

    int phase_;  // 0 = move, 1 = turn, 2 = move again
    const double target_distance_ = 0.2;  // Target distance to move (in meters)
    const double target_angle_ = M_PI / 2;  // Target angle to turn (90 degrees)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveAndTurnNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}