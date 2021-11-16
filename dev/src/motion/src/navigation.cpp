#include "custom_interfaces/msg/manual_control.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#define NO_BALL_IN_VIEW             -320
#define ANGULAR_VELOCITY_FACTOR     -0.01
#define MAX_SPEED                   0.91

class Navigation : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ball_direction_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
        rclcpp::Subscription<custom_interfaces::msg::ManualControl>::SharedPtr manual_control_subscriber;
        // Determines if manual control is enabled / if we need to stop.
        bool manual_control, stop;
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
            
            ball_direction_subscriber = create_subscription<std_msgs::msg::Int32>(
                "direction",
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
        void control_loop(const std_msgs::msg::Int32::SharedPtr ball_location)
        {
            if (stop)
            {
                publish_velocity();
            }
            else if (manual_control)
            {
                return;
            }
            else if (ball_location->data == NO_BALL_IN_VIEW)
            {

                double seconds = now().seconds() - time_since_last_seen.seconds();
                if (seconds > 2.5 && seconds < 10.0)
                {
                    publish_velocity(0, 1);
                }
                else
                {
                    TurnDirection angular = STRAIGHT;
                    publish_velocity(angular == STRAIGHT ? MAX_SPEED : 0, 1.15 * angular);
                }
            }
            else
            {
                time_since_last_seen = now();
                publish_velocity(0.8 * MAX_SPEED, ball_location->data * ANGULAR_VELOCITY_FACTOR);
            }
        }
/*
        TurnDirection determine_direction()
        {
            const double THRESHOLD = (FIELD_SIZE / 2) - 0.75;
            
            double theta = 0 // euler_from_quaternion(pose.orientation)['z'];
            if (pose.position.x > THRESHOLD)
            {
                RCLCPP_DEBUG(get_logger(), "Near x = %lf", THRESHOLD);
                // Facing 0
                if (theta >= 0 && theta < M_PI_2)
                    return LEFT;
                if (theta <= 0 && theta > -M_PI_2)
                    return RIGHT;
            }
            else if (pose.position.x < -THRESHOLD)
            {
                RCLCPP_DEBUG(get_logger(), "Near x = %lf", -THRESHOLD);
                // Facing PI
                if (theta <= M_PI && theta > M_PI_2)
                    return RIGHT;
                if (theta >= -M_PI && theta < -M_PI_2)
                    return LEFT;
            }
            else if (pose.position.y > THRESHOLD)
            {
                RCLCPP_DEBUG(get_logger(), "Near y = %lf", THRESHOLD);
                // Facing pi/2
                if (theta <= M_PI_2 && theta > 0)
                    return RIGHT;
                if (theta >= M_PI_2 && theta < M_PI)
                    return LEFT;
            }
            else if (pose.position.y < -THRESHOLD)
            {
                RCLCPP_DEBUG(get_logger(), "Near y = %lf", -THRESHOLD);
                // Facing -pi/2
                if (theta >= -M_PI_2 && theta < 0)
                    return LEFT;
                if (theta <= -M_PI_2 && theta >= -M_PI)
                    return RIGHT;
            }

            return STRAIGHT;
        }
*/
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
