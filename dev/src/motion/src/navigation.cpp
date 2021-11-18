#include <climits>

#include "custom_interfaces/msg/image_data.hpp"
#include "custom_interfaces/msg/manual_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#define NO_BALL_IN_VIEW             -320
#define NO_EDGE                     INT_MAX
#define ANGULAR_VELOCITY_FACTOR     -0.01
#define MAX_SPEED                   0.91


class Navigation : public rclcpp::Node
{
    private:
        // Determines if manual control is enabled / if we need to stop.
        bool manual_control, stop;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
        rclcpp::Subscription<custom_interfaces::msg::ImageData>::SharedPtr image_data_subscriber;
        rclcpp::Subscription<custom_interfaces::msg::ManualControl>::SharedPtr manual_control_subscriber;
        rclcpp::Time time_since_last_seen;
        enum TurnDirection {
            LEFT = 1,
            STRAIGHT = 0,
            RIGHT = -1
        };

    public:
        Navigation() : Node("navigation")
        {
            manual_control = false;
            stop = false;
            time_since_last_seen = rclcpp::Time(1000000);

            velocity_publisher = create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel",
                10
            );
            
            image_data_subscriber = create_subscription<custom_interfaces::msg::ImageData>(
                "image_data",
                5,
                std::bind(&Navigation::control_loop, this, std::placeholders::_1)
            );
            manual_control_subscriber = create_subscription<custom_interfaces::msg::ManualControl>(
                "navigation_control",
                10,
                std::bind(&Navigation::joy_control_handler, this, std::placeholders::_1)
            );

            RCLCPP_INFO(get_logger(), "%s node has started", get_name());
        }

    private:
        void control_loop(const custom_interfaces::msg::ImageData::SharedPtr image_data)
        {
            if (stop)
            {
                publish_velocity();
            }
            else if (manual_control)
            {
                // method that receives manual control calls publish_velocity()
                return;
            }
            else if (image_data->ball_position == NO_BALL_IN_VIEW)
            {
                double seconds = now().seconds() - time_since_last_seen.seconds();
                if (seconds > 1.5 && seconds < 9.0)
                {
                    publish_velocity(0, 1);
                }
                else
                {
                    TurnDirection direction = determine_direction(image_data->corner_position);
                    publish_velocity(direction == STRAIGHT ? MAX_SPEED : 0, direction);
                }
            }
            else
            {
                time_since_last_seen = now();
                publish_velocity(0.8 * MAX_SPEED, image_data->ball_position * ANGULAR_VELOCITY_FACTOR);
            }
        }

        TurnDirection determine_direction(int corner_position)
        {
            if (corner_position == NO_EDGE)
                return STRAIGHT;

            return (corner_position < 0) ? LEFT : RIGHT;
        }

        void publish_velocity(double linear = 0, double angular = 0)
        {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = linear;
            message.angular.z = angular;
            velocity_publisher->publish(message);
        }

        void joy_control_handler(const custom_interfaces::msg::ManualControl::SharedPtr message)
        {
            if (message->stop)
            {
                stop = true;
                manual_control = false;
            }
            else
            {
                float linear_percentage = message->linear_percentage;
                float angular_percentage = message->angular_percentage;

                if (abs(linear_percentage) < 1.0e-6 && abs(angular_percentage) < 1.0e-6)
                {
                    stop = false;
                    manual_control = false;
                }
                else
                {
                    stop = false;
                    manual_control = true;
                    publish_velocity(MAX_SPEED * linear_percentage, angular_percentage);
                }
            }
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Navigation>());
    rclcpp::shutdown();
    return 0;
}
