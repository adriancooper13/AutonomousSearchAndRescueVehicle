#include "custom_interfaces/msg/manual_control.hpp"
#include "custom_interfaces/msg/threshold_adjustment.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int8.hpp"

#define OFF false
#define ON  true

// ================================
// MAPPINGS ARE WITH PS4 CONTROLLER
// ================================
const std::map<std::string, int> BUTTON_MAPPINGS = {
    { "X", 0 },
    { "CIRCLE", 1 },
    { "O", 1 },
    { "TRIANGLE", 2 },
    { "SQUARE", 3 },
    { "L1", 4 },
    { "R1", 5 },
    { "L2", 6 },
    { "R2", 7 },
    { "OPTION", 8 },
    { "SHARE", 9 },
    { "PS", 10 },
    { "L3", 11 },
    { "R3", 12 }
};

const std::map<std::string, int> AXES_MAPPINGS = {
    { "L3-L/R", 0 },
    { "L3-U/D", 1 },
    { "L2", 2 },
    { "R3-L/R", 3 },
    { "R3-U/D", 4 },
    { "R2", 5 },
    { "DPAD-L/R", 6 },
    { "DPAD-U/D", 7 }
};

class ManualControl : public rclcpp::Node
{
    private:
        rclcpp::Publisher<custom_interfaces::msg::ManualControl>::SharedPtr control_publisher;
        rclcpp::Publisher<custom_interfaces::msg::ThresholdAdjustment>::SharedPtr vision_adjustment_publisher;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;

    public:
        ManualControl() : Node("manual_control")
        {
            joy_subscriber = create_subscription<sensor_msgs::msg::Joy>(
                "joy",
                10,
                std::bind(&ManualControl::joy_publisher, this, std::placeholders::_1)
            );
            control_publisher = create_publisher<custom_interfaces::msg::ManualControl>(
                "navigation_control",
                10
            );
            vision_adjustment_publisher = create_publisher<custom_interfaces::msg::ThresholdAdjustment>(
                "vision_threshold_adjustment",
                10
            );

            RCLCPP_INFO(get_logger(), "%s node has started", get_name());
        }

    private:
        void joy_publisher(const sensor_msgs::msg::Joy::SharedPtr input)
        {   
            publish_control(input);
            publish_vision_adjust(input);
        }

        void publish_control(const sensor_msgs::msg::Joy::SharedPtr input)
        {
            auto message = custom_interfaces::msg::ManualControl();

            message.stop = read_stop(input);
            message.linear_percentage = calculate_linear_percentage(input);
            message.angular_percentage = calculate_angular_percentage(input);

            control_publisher->publish(message);
        }

        void publish_vision_adjust(const sensor_msgs::msg::Joy::SharedPtr input)
        {
            auto message = custom_interfaces::msg::ThresholdAdjustment();
            
            int adjustment = input->axes.at(AXES_MAPPINGS.at("DPAD-U/D"));
            if (input->buttons.at(BUTTON_MAPPINGS.at("L1")) == ON)
                message.lower_adjustment = adjustment;
            if (input->buttons.at(BUTTON_MAPPINGS.at("R1")) == ON)
                message.upper_adjustment = adjustment;
            
            vision_adjustment_publisher->publish(message);
        }

        double calculate_linear_percentage(const sensor_msgs::msg::Joy::SharedPtr input)
        {
            bool forward = input->buttons.at(BUTTON_MAPPINGS.at("R2"));
            bool backward = input->buttons.at(BUTTON_MAPPINGS.at("L2"));

            double linear;

            // Both forward and backward button pressed or neither being pressed.
            // Equivalent to: (forward == ON && backward == ON) || (forward == OFF && backward == OFF)
            if (forward == backward)
            {
                // Do not go forward or backward.
                linear = 0;
            }
            // Forward button only being pressed.
            else if (forward == ON)
            {
                linear = 1 - input->axes.at(AXES_MAPPINGS.at("R2"));
            }
            // Backward button only being pressed.
            else
            {
                linear = -1 + input->axes.at(AXES_MAPPINGS.at("L2"));
            }

            // Get percentage. Range of possible values is from [-1, 1]
            return linear / 2;
        }

        double calculate_angular_percentage(const sensor_msgs::msg::Joy::SharedPtr input)
        {
            double angular = input->axes.at(AXES_MAPPINGS.at("DPAD-L/R"));

            // If the user is holding dpad and L3, add results together.
            angular += input->axes.at(AXES_MAPPINGS.at("L3-L/R"));

            // Get percentage.
            // Range of possible values from d-pad is [-1, 1]
            // Range of possible values from L3 is [-1, 1]
            // Added together values give possible range of [-2, 2]
            // Governing turning speed range of [-pi, pi] rad/s
            return angular * M_PI / 2;
        }

        bool read_stop(const sensor_msgs::msg::Joy::SharedPtr input)
        {
            return input->buttons.at(BUTTON_MAPPINGS.at("CIRCLE")) == ON;
        }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControl>());
    rclcpp::shutdown();
    return 0;
}
