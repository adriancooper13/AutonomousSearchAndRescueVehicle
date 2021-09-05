#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PathPlanning : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;

    public:
        PathPlanning() : Node("PathPlanning")
        {
            subscription = this->create_subscription<std_msgs::msg::String>(
                "Sight",
                10,
                std::bind(
                    &PathPlanning::topic_callback,
                    this,
                    std::placeholders::_1
                )
            );
        }

    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
        {
            RCLCPP_INFO(
                this->get_logger(),
                "I heard: '%s'",
                msg->data.c_str()
            );
        }
};