#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node 
{
    private:
        rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher;
        rclcpp::TimerBase::SharedPtr timer;

    public:
        AddressBookPublisher() : Node("address_book_publisher")
        {
            address_book_publisher =
                this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

            auto publish_msg = [this]() -> void
            {
                auto message = more_interfaces::msg::AddressBook();

                message.first_name = "John";
                message.last_name = "Doe";
                message.age = 30;
                message.gender = message.MALE;
                message.address = "unknown";

                std::cout << "Publishing Contact" << std::endl
                          << "First: " << message.first_name << ", Last: " << message.last_name
                          << std::endl;

                address_book_publisher->publish(message);
            };

            timer = this->create_wall_timer(1s, publish_msg);
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddressBookPublisher>());
    rclcpp::shutdown();
    return 0;
}