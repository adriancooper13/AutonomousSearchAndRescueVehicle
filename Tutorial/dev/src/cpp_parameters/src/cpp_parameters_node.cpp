#include <chrono>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::string;

class ParametersClass : public rclcpp::Node
{
    private:
        string parameter_string;
        rclcpp::TimerBase::SharedPtr timer;

    public:
        ParametersClass() : Node("parameter_node")
        {
            this->declare_parameter<string>("my_parameter", "world");
            timer = this->create_wall_timer(
                1000ms,
                std::bind(&ParametersClass::respond, this)
            );
        }

        void respond()
        {
            this->get_parameter("my_parameter", parameter_string);
            RCLCPP_INFO(
                this->get_logger(),
                "Hello %s",
                parameter_string.c_str()
            );
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParametersClass>());
    rclcpp::shutdown();
    return 0;
}