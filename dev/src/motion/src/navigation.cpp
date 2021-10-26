#include "gazebo_msgs/srv/delete_entity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "../../Golfball.hpp"
#include "../../helpers.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#define NO_BALL_IN_VIEW             -180
#define ANGULAR_VELOCITY_FACTOR     -0.01
#define MAX_SPEED                   0.91
#define FIELD_SIZE                  22.86

class Navigation : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ball_direction_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_control_subscriber;
        // Current pose for finding the closest golfball.
        geometry_msgs::msg::Pose pose;
        // Determines if manual control is enabled.
        bool manual_control;

        enum TurnDirection {
            LEFT = 1,
            STRAIGHT = 0,
            RIGHT = -1
        };

    public:
        Navigation() : Node("navigation_node")
        {
            manual_control = false;

            velocity_publisher = create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel",
                1
            );

            ball_direction_subscriber = create_subscription<std_msgs::msg::Int32>(
                "direction",
                10,
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

            RCLCPP_INFO(get_logger(), "%s node has started", get_name());
        }

    private:
        void control_loop(const std_msgs::msg::Int32::SharedPtr ball_location)
        {
            if (manual_control)
                return;
            
            if (ball_location->data == NO_BALL_IN_VIEW)
            {
                TurnDirection angular = determine_direction();
                publish_velocity(angular == STRAIGHT ? MAX_SPEED : 0.5 * MAX_SPEED, angular);
            }
            else
            {
                publish_velocity(0.5 * MAX_SPEED, ball_location->data * ANGULAR_VELOCITY_FACTOR);
            }
        }

        TurnDirection determine_direction()
        {
            const double THRESHOLD = (FIELD_SIZE / 2) - 1.5;
            
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

        void publish_velocity(double linear, double angular = 0)
        {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = linear;
            message.angular.z = angular;
            velocity_publisher->publish(message);
        }

        // int closest_golfball_index()
        // {
        //     int size = golfballs->size();
        //     if (size == 0)
        //         return -1;

        //     int closest_index = 0;
        //     double best_dist = golfballs->at(0).distance(pose.position.x, pose.position.y);

        //     for (int i = 1; i < size; i++)
        //     {
        //         double dist = golfballs->at(i).distance(pose.position.x, pose.position.y);
        //         if (dist < best_dist)
        //         {
        //             best_dist = dist;
        //             closest_index = i;
        //         }
        //     }

        //     return closest_index;
        // }

        void joy_control_handler(const geometry_msgs::msg::Twist::SharedPtr message)
        {
            if (message->linear.x == 0 && message->angular.z == 0)
            {
                manual_control = false;
                return;
            }

            manual_control = true;
            publish_velocity(message->linear.x, message->angular.z);
        }

        void get_current_position(const nav_msgs::msg::Odometry::SharedPtr message)
        {
            // message->pose has type PoseWithCovariance. Tack on .pose to extract just pose
            pose = message->pose.pose;
        }

        // bool close_enough(int index)
        // {
        //     double dist = golfballs->at(index).distance(
        //         pose.position.x,
        //         pose.position.y
        //     );

        //     return dist < 0.2;
        // }

        // int close_enough()
        // {
        //     int size = golfballs->size();
        //     for (int i = 0; i < size; i++)
        //     {
        //         if (golfballs->at(i).distance(pose.position.x, pose.position.y) < 0.2)
        //             return i;
        //     }

        //     return -1;
        // }

        // void remove_golfball(int index)
        // {
        //     auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        //     request->name = golfballs->at(index).NAME;
        //     golfballs->erase(golfballs->begin() + index);

        //     auto client = create_client<gazebo_msgs::srv::DeleteEntity>("delete_entity");
        //     while (!client->wait_for_service(1s))
        //     {
        //         RCLCPP_WARN(
        //             get_logger(),
        //             "%s service not available. Waiting...",
        //             client->get_service_name()
        //         );
        //     }

        //     auto future = client->async_send_request(request);
        //     try
        //     {
        //         auto response = future.get();
        //         RCLCPP_INFO(get_logger(), "Deleted golfball: %s", response->success ? "true" : "false");
        //     }
        //     catch (const std::exception &e)
        //     {
        //         RCLCPP_WARN(get_logger(), "Service call failed. Exception %s", e.what());
        //     }
        // }

        // void retrieve_golfballs()
        // {
        //     if (manual_control)
        //     {
        //         int index = close_enough();
        //         if (index != -1)
        //             std::thread(std::bind(&Navigation::remove_golfball, this, index)).detach();
        //         return;
        //     }

        //     int index = closest_golfball_index();
        //     if (index == -1)
        //     {
        //         golfballs->push_back(Golfball(11, -11, "home"));
        //         if (!close_enough(0))
        //             send_goal(0);
        //         return;
        //     }

        //     send_goal(index);
        //     if (close_enough(index))
        //     {
        //         std::thread(std::bind(&Navigation::remove_golfball, this, index)).detach();
        //     }
        // }

        // void send_goal(int index)
        // {
        //     auto rpy = euler_from_quaternion(pose.orientation);
        //     double angle = atan2(
        //         golfballs->at(index).Y - pose.position.y,
        //         golfballs->at(index).X - pose.position.x
        //     );
    
        //     angle -= rpy['z'];
        //     if (angle > M_PI)
        //         angle -= 2 * M_PI;
        //     else if (angle < -M_PI)
        //         angle += 2 * M_PI;
            
        //     double abs_angle = fabs(angle);
        //     double dist = golfballs->at(index).distance(pose.position.x, pose.position.y);
            
        //     RCLCPP_DEBUG(
        //         get_logger(),
        //         "Angle w/o orientation: %lf, Yaw: %lf, Angle: %lf",
        //         angle + rpy['z'],
        //         rpy['z'],
        //         angle
        //     );

        //     auto message = geometry_msgs::msg::Twist();
        //     if (abs_angle * 180 / M_PI > 90)
        //     {
        //         message.linear.x = 0;
        //         message.angular.z = 0.5 * angle;
        //     }
        //     else if (dist > 10)
        //     {
        //         message.linear.x = 0.1 * dist;
        //         message.angular.z = 0.5 * angle;
        //     }
        //     else if (dist > 5)
        //     {
        //         message.linear.x = 0.25 * dist;
        //         message.angular.z = 0.3 * angle;
        //     }
        //     else if (dist > 2.5)
        //     {
        //         message.linear.x = 0.5 * dist;
        //         message.angular.z = (abs_angle > 0.3) ? 0.5 * angle : 0.3 * angle;
        //     }
        //     else
        //     {
        //         message.linear.x = (abs_angle > 0.3) ? 0 : dist;
        //         message.angular.z = 0.5 * angle;
        //     }
        //     goal_publisher->publish(message);
        // }

        // void get_golfball_locations(
        //     const std::shared_ptr<custom_interfaces::srv::TransferGolfballLocations::Request> request,
        //     const std::shared_ptr<custom_interfaces::srv::TransferGolfballLocations::Response> response)
        // {
        //     if (request->xs.size() != request->ys.size())
        //     {
        //         response->success = false;
        //         return;
        //     }
        //     int size = request->xs.size();
        //     if (size != (int)request->names.size())
        //     {
        //         response->success = false;
        //         return;
        //     }

        //     golfballs = new std::vector<Golfball>();
        //     for (int i = 0; i < size; i++)
        //     {
        //         golfballs->push_back(
        //             Golfball(
        //                 request->xs.at(i),
        //                 request->ys.at(i),
        //                 request->names.at(i)
        //         ));
        //     }

        //     response->success = true;
        //     RCLCPP_INFO(get_logger(), "Received golfball locations");
        // }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Navigation>());
    rclcpp::shutdown();
    return 0;
}