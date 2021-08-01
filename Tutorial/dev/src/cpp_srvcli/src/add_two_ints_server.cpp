#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std;

void add(
    const shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "Incoming request [a: %ld, b: %ld]",
        request->a,
        request->b
    );
    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "Sending back response: [%ld]",
        (long)response->sum
    );
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
        node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}