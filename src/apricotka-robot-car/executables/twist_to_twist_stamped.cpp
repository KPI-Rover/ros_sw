#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TwistToTwistStampedNode : public rclcpp::Node
{
public:
    TwistToTwistStampedNode() : Node("twist_to_twist_stamped_node")
    {
        input_vel_ = this->declare_parameter("input_vel", "/cmd_vel");
        output_vel_ = this->declare_parameter("output_vel", "/cmd_stamped_vel");
        frame_id_ = this->declare_parameter("frame_id", "");

        // Subscriber to input_vel_
        input_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(input_vel_, 10,
        std::bind(&TwistToTwistStampedNode::inputVelCallback, this, std::placeholders::_1));
        
        // Publisher to output_vel_
        output_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_vel_, 10);
        
        RCLCPP_INFO(this->get_logger(), "TwistToTwistStampedNode has been started");
    }

private:
    std::string input_vel_;
    std::string output_vel_;
    std::string frame_id_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_vel_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_vel_publisher_;

    void inputVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
        twist_stamped_msg.header.stamp = this->get_clock()->now();
        twist_stamped_msg.header.frame_id = frame_id_;
        twist_stamped_msg.twist = *msg;
        
        // Publish the TwistStamped message to /base_velocity
        output_vel_publisher_->publish(twist_stamped_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToTwistStampedNode>());
    rclcpp::shutdown();
    return 0;
}
