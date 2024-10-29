#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

class MoveNode : public rclcpp::Node
{
public:
    MoveNode() : Node("move_node"), distance_moved_(0.0), initial_x_(0.0), initial_y_(0.0)
    {
        move_distance_ = this->declare_parameter("move_distance", 1.0);
        move_speed_ = this->declare_parameter("move_speed", 0.5);

        // Publisher to send velocity commands with TwistStamped
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);

        // Subscriber to odometry data
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diff_drive_controller/odom", 10, std::bind(&MoveNode::odomCallback, this, std::placeholders::_1));

        // Timer to control the robot
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MoveNode::controlLoop, this));
    }

private:
    double move_distance_;
    double move_speed_;
    double distance_moved_;
    double current_x_, current_y_;
    double initial_x_, initial_y_;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Odometry callback to track position
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Calculate the distance moved
        distance_moved_ = std::sqrt(std::pow(current_x_ - initial_x_, 2) +
                                    std::pow(current_y_ - initial_y_, 2));
    }

    void controlLoop()
    {
        geometry_msgs::msg::TwistStamped cmd_msg;
        cmd_msg.header.stamp = this->get_clock()->now();
        cmd_msg.header.frame_id = "base_link";  // Assuming the base frame is "base_link"

        if (distance_moved_ < move_distance_)
        {
            cmd_msg.twist.linear.x = move_speed_;  // Move forward
        }
        else
        {
            cmd_msg.twist.linear.x = 0.0;  // Stop moving forward
            RCLCPP_INFO(this->get_logger(), "Movement completed!");
        }
        
        // Publish the velocity command
        cmd_vel_pub_->publish(cmd_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}