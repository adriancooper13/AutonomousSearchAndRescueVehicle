#include "diff_drive_controller/diff_drive_controller.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#define NO_BALL_IN_VIEW             -180
#define ANGULAR_VELOCITY_FACTOR     -0.01
#define MAX_SPEED                   0.91
#define FIELD_SIZE                  22.86
#define WHEEL_SEPARATION            0.287
#define WHEEL_RADIUS                0.033

std::map<char, double> euler_from_quaternion(geometry_msgs::msg::Quaternion orientation)
{
    auto rpy = std::map<char, double>();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (orientation.w * orientation.x + orientation.y * orientation.z);
    double cosr_cosp = 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y);
    rpy['x'] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (orientation.w * orientation.y - orientation.z * orientation.x);
    if (std::abs(sinp) >= 1)
        rpy['y'] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        rpy['y'] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
    double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
    rpy['z'] = std::atan2(siny_cosp, cosy_cosp);

    return rpy;
}

geometry_msgs::msg::Quaternion quaternion_from_euler(double roll, double pitch, double yaw)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    auto q = geometry_msgs::msg::Quaternion();
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    q.w = cy * cp * cr + sy * sp * sr;

    return q;
}

class Navigation : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ball_direction_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_control_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
        // Current pose for detecting edges.
        geometry_msgs::msg::Pose pose;
        // Determines if manual control is enabled / if we need to stop.
        bool manual_control, stop;


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

            velocity_publisher = create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel",
                10
            );
            
            ball_direction_subscriber = create_subscription<std_msgs::msg::Int32>(
                "direction",
                5,
                std::bind(&Navigation::control_loop, this, std::placeholders::_1)
            );
            odom_subscriber = create_subscription<nav_msgs::msg::Odometry>(
                "odom", 
                10,
                std::bind(&Navigation::get_current_position, this, std::placeholders::_1)
            );
            manual_control_subscriber = create_subscription<geometry_msgs::msg::Twist>(
                "joy_control",
                10,
                std::bind(&Navigation::joy_control_handler, this, std::placeholders::_1)
            );
            stop_subscriber = create_subscription<std_msgs::msg::Bool>(
                "stop",
                1,
                std::bind(&Navigation::stop_handler, this, std::placeholders::_1)
            );

            RCLCPP_INFO(get_logger(), "%s node has started", get_name());
        }

    private:
        void control_loop(const std_msgs::msg::Int32::SharedPtr ball_location)
        {
            if (manual_control)
                return;
            
            if (stop)
            {
                publish_velocity();
            }
            else if (ball_location->data == NO_BALL_IN_VIEW)
            {
                TurnDirection angular = determine_direction();
                publish_velocity(angular == STRAIGHT ? MAX_SPEED : 0, 1.15 * angular);
            }
            else
            {
                publish_velocity(0.8 * MAX_SPEED, ball_location->data * ANGULAR_VELOCITY_FACTOR);
            }
        }

        TurnDirection determine_direction()
        {
            const double THRESHOLD = (FIELD_SIZE / 2) - 0.75;
            
            double theta = euler_from_quaternion(pose.orientation)['z'];
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

        void publish_velocity(double linear = 0, double angular = 0)
        {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = linear;
            message.angular.z = angular;
            velocity_publisher->publish(message);
        }

        void joy_control_handler(const geometry_msgs::msg::Twist::SharedPtr message)
        {
            if (message->linear.x == 0 && message->angular.z == 0)
            {
                manual_control = false;
                return;
            }

            manual_control = true;
            publish_velocity(MAX_SPEED * message->linear.x / 2, message->angular.z);
        }

        void stop_handler(const std_msgs::msg::Bool::SharedPtr message)
        {
            stop = message->data;
        }

        void get_current_position(const nav_msgs::msg::Odometry::SharedPtr message)
        {
            // message->pose has type PoseWithCovariance. Tack on .pose to extract just pose
            pose = message->pose.pose;
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Navigation>());
    rclcpp::shutdown();
    return 0;
}