#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"

using namespace std;

void add(
    const shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,
    shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response> response)
{
    response->sum = request->a + request->b + request->c;
    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "Incoming request [a: %ld, b: %ld, c: %ld]",
        request->a,
        request->b,
        request->c
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

    rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =
        node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_two_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}