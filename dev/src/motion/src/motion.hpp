#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class Motion : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;

    public:
        Motion() : Node("motion_publisher")
        {
            publisher = create_publisher<geometry_msgs::msg::Twist>(
                "turtle1/cmd_vel",
                10
            );
            timer = create_wall_timer(
                500ms,
                std::bind(&Motion::timer_callback, this)
            );
        }

    private:
        void timer_callback()
        {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = 0.5;
            RCLCPP_INFO(
                get_logger(),
                "Moving at a rate of %lf in the x direction",
                message.linear.x
            );
            publisher->publish(message);
        }
};