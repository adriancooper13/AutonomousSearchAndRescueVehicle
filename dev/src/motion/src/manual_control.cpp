#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class ManualControl : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

    public:
        ManualControl() : Node("manual_control")
        {
            joy_subscriber = create_subscription<sensor_msgs::msg::Joy>(
                "message_name", // TODO: add message name
                10,
                std::bind(&ManualControl::joy_publisher, this, std::placeholders::_1)
            );
            RCLCPP_INFO(get_logger(), "%s node has started", get_name());
        }

    private:
        void joy_publisher(const sensor_msgs::msg::Joy::SharedPtr input)
        {
            RCLCPP_INFO(get_logger(), "Add something here");
        }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControl>());
    rclcpp::shutdown();
    return 0;
}