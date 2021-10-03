#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class Controller : public rclcpp::Node
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr goal_subscriber;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;

    public:
        Controller() : Node("controller")
        {
            goal_subscriber = create_subscription<geometry_msgs::msg::Twist>(
                "nav_goal",
                10,
                std::bind(&Controller::go_to_position, this, std::placeholders::_1)
            );

            cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel",
                10
            );

            RCLCPP_INFO(get_logger(), "%s node started", get_name());
        }

    private:
        void go_to_position(const geometry_msgs::msg::Twist::SharedPtr goal)
        {   
            // TODO: this will eventually output to the arduino
            cmd_vel_publisher->publish(*goal);
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}