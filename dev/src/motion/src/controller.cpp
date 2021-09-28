#include <climits>

#include "custom_interfaces/srv/transfer_golfball_locations.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#define FIELD_SIZE 22.86 // 75ft = 22.86m

using namespace std::chrono_literals;

double distance(const std::pair<double, double> &curr, const std::pair<double, double> &pos)
{
    return sqrt(pow(pos.first - curr.first, 2) + pow(pos.second - curr.second, 2));
}

class Golfball
{
    double x;
    double y;
    std::string name;
    int index;

    public:
        Golfball(double x, double y, std::string name, int index)
        {
            this->x = x;
            this->y = y;
            this->name = name;
            this->index = index;
        }

        double get_x() { return x; }
        double get_y() { return y; }
        double get_index() { return index; }
};

class ControllerNode : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
        rclcpp::Service<custom_interfaces::srv::TransferGolfballLocations>::SharedPtr golfball_locations_service;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
        rclcpp::TimerBase::SharedPtr timer;
        // Point represents the golfballs position, name represents the entity name for deleting.
        std::vector<std::pair<geometry_msgs::msg::Point, std::string>> *golfballs;
        // Current pose for finding the closest golfball.
        geometry_msgs::msg::Pose pose;

    public:
        ControllerNode() : Node("controller_node")
        {
            cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            timer = create_wall_timer(
                0.01s,
                std::bind(&ControllerNode::control_loop, this)
            );
            odom_subscriber = create_subscription<nav_msgs::msg::Odometry>(
                "odom", 
                10,
                std::bind(&ControllerNode::get_current_position, this, std::placeholders::_1)
            );
            pose = geometry_msgs::msg::Pose();
            pose.position.x = 11;
            pose.position.y = -11;
            
            golfballs = nullptr;
            golfball_locations_service = create_service<custom_interfaces::srv::TransferGolfballLocations>(
                "/golfball_locations",
                std::bind(&ControllerNode::get_golfball_locations, this, std::placeholders::_1, std::placeholders::_2)
            );

            RCLCPP_INFO(get_logger(), "%s has started", get_name());
        }

        ~ControllerNode()
        {
            delete golfballs;
        }

    private:
        void control_loop()
        {
            if (golfballs != nullptr)
                retrieve_golfballs();
        }

        Golfball *find_closest_golfball()
        {
            int size = golfballs->size();
            if (size == 0)
                return nullptr;

            int closest_index = 0;
            double best_dist = distance(
                std::make_pair(golfballs->at(closest_index).first.x, golfballs->at(closest_index).first.y),
                std::make_pair(pose.position.x, pose.position.y)
            );

            for (int i = 1; i < size; i++)
            {
                double dist = distance(
                    std::make_pair(golfballs->at(i).first.x, golfballs->at(i).first.y),
                    std::make_pair(pose.position.x, pose.position.y)
                );
                if (dist < best_dist)
                {
                    best_dist = dist;
                    closest_index = i;
                }
            }

            return new Golfball(
                golfballs->at(closest_index).first.x,
                golfballs->at(closest_index).first.y,
                golfballs->at(closest_index).second,
                closest_index
            );
        }

        void get_current_position(const nav_msgs::msg::Odometry::SharedPtr message)
        {
            // message->pose has type PoseWithCovariance. Tack on .pose to extract just pose
            pose = message->pose.pose;
        }

        std::map<char, double> euler_from_quaternion()
        {
            auto rpy = std::map<char, double>();
            auto q = pose.orientation;

            // roll (x-axis rotation)
            double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
            double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
            rpy['x'] = std::atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            double sinp = 2 * (q.w * q.y - q.z * q.x);
            if (std::abs(sinp) >= 1)
                rpy['y'] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            else
                rpy['y'] = std::asin(sinp);

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            rpy['z'] = std::atan2(siny_cosp, cosy_cosp);

            return rpy;
        }

        void go_to_position(double x, double y)
        {
            auto rpy = euler_from_quaternion();
            auto angle = atan2(y - pose.position.y, x - pose.position.x) - rpy['z'];
            auto abs_angle = fabs(angle);
            auto dist = distance(
                std::make_pair(pose.position.x, pose.position.y),
                std::make_pair(x, y)
            );
            
            RCLCPP_WARN(get_logger(), "%lf %lf", dist, angle);
            
            auto message = geometry_msgs::msg::Twist();
            if (dist > 5 && abs_angle > 0.1)
            {
                message.linear.x = 0.5 * dist;
                message.angular.z = 0.2 * angle;
            }
            else if (dist > 5)
            {
                message.linear.x = 0.5 * dist;
                message.angular.z = 0;
            }
            else if (abs_angle > 0.1)
            {
                message.linear.x = 0;
                message.angular.z = 0.3 * angle;
            }
            else
            {
                message.linear.x = 0.3;
                message.angular.z = 0;
            }
            cmd_vel_publisher->publish(message);
        }

        bool close_enough(double x, double y)
        {
            double dist = distance(
                std::make_pair(pose.position.x, pose.position.y),
                std::make_pair(x, y)
            );

            return dist < 0.3;
        }

        void remove_golfball(int index)
        {
            auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
            request->name = golfballs->at(index).second;
            golfballs->erase(golfballs->begin() + index);

            auto client = create_client<gazebo_msgs::srv::DeleteEntity>("delete_entity");
            while (!client->wait_for_service(1s))
            {
                RCLCPP_WARN(
                    get_logger(),
                    "%s service not available. Waiting...",
                    client->get_service_name()
                );
            }

            auto future = client->async_send_request(request);
            try
            {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "Deleted golfball: %s", response->success ? "true" : "false");
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "Service call failed. Exception %s", e.what());
            }
        }

        void retrieve_golfballs()
        {
            auto closest = find_closest_golfball();
            if (closest == nullptr)
            {
                go_to_position(11, 11);
                return;
            }

            double x = closest->get_x(), y = closest->get_y();
            int index = closest->get_index();
            go_to_position(x, y);
            if (close_enough(x, y))
            {
                std::thread(std::bind(&ControllerNode::remove_golfball, this, index)).detach();
            }

            delete closest;
        }

        void get_golfball_locations(
            const std::shared_ptr<custom_interfaces::srv::TransferGolfballLocations::Request> request,
            const std::shared_ptr<custom_interfaces::srv::TransferGolfballLocations::Response> response)
        {
            if (request->xs.size() != request->ys.size())
            {
                response->success = false;
                return;
            }
            int size = request->xs.size();
            if (size != (int)request->names.size())
            {
                response->success = false;
                return;
            }

            golfballs = new std::vector<std::pair<geometry_msgs::msg::Point, std::string>>();
            for (int i = 0; i < size; i++)
            {
                geometry_msgs::msg::Point point = geometry_msgs::msg::Point();
                point.x = request->xs.at(i);
                point.y = request->ys.at(i);

                golfballs->push_back(std::make_pair(point, request->names.at(i)));
            }

            response->success = true;
            RCLCPP_INFO(get_logger(), "Received golfball locations");
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}