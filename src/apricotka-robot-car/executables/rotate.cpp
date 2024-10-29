#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class RotateNode : public rclcpp::Node
{
public:
    RotateNode() : Node("rotate_node")
    {
        // Get the rotation angle and speed from parameters
        rotation_angle_ = this->declare_parameter("rotation_angle", 90.0);  // Example: rotate 90 degrees
        rotation_speed_ = this->declare_parameter("rotation_speed", 0.5);   // Example: 0.5 rad/s

        // Subscriber for IMU
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&RotateNode::imu_callback, this, std::placeholders::_1));

        // Publisher for velocity commands
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);

        // Timer to regularly check the yaw and issue velocity commands
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RotateNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Rotate robot node started, rotation angle: %.2f, rotation speed: %.2f", rotation_angle_, rotation_speed_);
    }

private:
    double rotation_angle_;  // Angle to rotate relative to the current yaw
    double rotation_speed_;  // Rotation speed (angular velocity in rad/s)
    double target_yaw_;
    double current_yaw_ = 0.0;
    bool has_imu_data_ = false;
    bool target_yaw_set_ = false;  // Flag to set target yaw only once when starting rotation

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback for IMU data
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract the yaw (z) from the quaternion in the IMU data
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        // Convert yaw from radians to degrees and normalize it to [0, 360)
        current_yaw_ = fmod((yaw * 180.0 / M_PI) + 360.0, 360.0);

        has_imu_data_ = true;

        // Set the target yaw only once after receiving initial IMU data
        if (!target_yaw_set_) {
            target_yaw_ = fmod((current_yaw_ + rotation_angle_) + 360.0, 360.0);
            target_yaw_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Start yaw: %.2f", current_yaw_);
            RCLCPP_INFO(this->get_logger(), "Target yaw: %.2f", target_yaw_);
        }
    }

    // Control loop to rotate the robot
    void control_loop()
    {
        if (!has_imu_data_) {
            RCLCPP_WARN(this->get_logger(), "No IMU data yet");
            return;
        }

        double error = fmod(target_yaw_ - current_yaw_ + 360.0, 360.0);
        if (error > 180.0) {
            error -= 360.0;
        }

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->get_clock()->now();  // Add timestamp

        if (fabs(error) > 0.5)  // If error is greater than 0.5 degree, keep rotating
        {
            double angular_speed = rotation_speed_ * (error > 0 ? 1.0 : -1.0);  // Rotate clockwise or anticlockwise
            cmd_vel.twist.angular.z = angular_speed;
            vel_publisher_->publish(cmd_vel);
        }
        else // Stop and only send message when target is reached
        {
            RCLCPP_INFO(this->get_logger(), "Reached target yaw: %.2f", target_yaw_);
            cmd_vel.twist.angular.z = 0.0;
            vel_publisher_->publish(cmd_vel);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RotateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
