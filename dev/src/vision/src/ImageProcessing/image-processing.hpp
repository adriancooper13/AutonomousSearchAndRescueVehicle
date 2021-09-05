#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ImageProcessing : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        int count;

    public:
        ImageProcessing() : Node("ImageProcessing")
        {
            count = 0;
            publisher = this->create_publisher<std_msgs::msg::String>("Sight", 10);
            timer = this->create_wall_timer(
                500ms,
                std::bind(&ImageProcessing::timer_callback, this)
            );
        }

    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "I saw a golf ball";
            count += 1;

            RCLCPP_INFO(
                this->get_logger(),
                "Publishing: '%s'",
                message.data.c_str()
            );

            publisher->publish(message);
        }
};