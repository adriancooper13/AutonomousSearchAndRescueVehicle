#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define OFF false
#define ON  true

// ================================
// MAPPINGS ARE WITH PS4 CONTROLLER
// ================================
// Buttons:
    //  0: circle
    //  1:
    //  2: X
    //  3: triangle
    //  4: square
    //  5: 
    //  6: L2 default 0, pressed 1 discrete
    //  7: R2 default 0, pressed 1 discrete
    //  8: PS
    //  9: share
    //  10: option
const std::map<std::string, int> BUTTON_MAPPINGS = {
    { "CIRCLE", 0 },
    { "O", 0 },
    { "X", 2 },
    { "TRIANGLE", 3 },
    { "SQUARE", 4 },
    { "L2", 6 },
    { "R2", 7 },
    { "PS", 8 },
    { "SHARE", 9 },
    { "OPTION", 10 }
};

// Axes:
    //  0-2: left joy
    //  2, 5: right joy
    //  3: L2 (default 1, fully pressed -1) continuous
    //  4: R2 (default 1, fully pressed -1) continuous
    //  6: left/right
    //  7: up/down
const std::map<std::string, int> AXES_MAPPINGS = {
    // { "LEFT-JOY", 0, 1, 2 }, // FIXME: figure out what parts of left and right joy map to which parts of the array
    // { "RIGHT-JOY", 2, 5 },
    { "L2", 3 },
    { "R2", 4 },
    { "LEFT/RIGHT-DPAD", 6 },
    { "UP/DOWN-DPAD", 7 }
};

class ManualControl : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

    public:
        ManualControl() : Node("manual_control")
        {
            joy_subscriber = create_subscription<sensor_msgs::msg::Joy>(
                "joy",
                10,
                std::bind(&ManualControl::joy_publisher, this, std::placeholders::_1)
            );
            cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>(
                "joy_control",
                10
            );

            RCLCPP_INFO(get_logger(), "%s node has started", get_name());
        }

    private:
        void joy_publisher(const sensor_msgs::msg::Joy::SharedPtr input)
        {   
            // Go home.
            if (input->buttons.at(BUTTON_MAPPINGS.at("PS")) == ON)
            {
                go_home();
                return;
            }

            bool forward = input->buttons.at(BUTTON_MAPPINGS.at("R2"));
            bool backward = input->buttons.at(BUTTON_MAPPINGS.at("L2"));

            auto message = geometry_msgs::msg::Twist();
            // Both forward and backward button pressed or neither being pressed.
            // Equivalent to: (forward == ON && backward == ON) || (forward == OFF && backward == OFF)
            if (forward == backward)
            {
                // Do not go forward or backward.
                message.linear.x = 0;
            }
            // Forward button only being pressed.
            else if (forward == ON)
            {
                message.linear.x = 1 - input->buttons.at(AXES_MAPPINGS.at("R2"));
            }
            // Backward button only being pressed.
            else
            {
                message.linear.x = -1 + input->buttons.at(AXES_MAPPINGS.at("L2"));
            }

            message.angular.z = input->buttons.at(AXES_MAPPINGS.at("LEFT/RIGHT-DPAD"));
            cmd_vel_publisher->publish(message);
        }

        void go_home()
        {
            // TODO: write me :)
        }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControl>());
    rclcpp::shutdown();
    return 0;
}