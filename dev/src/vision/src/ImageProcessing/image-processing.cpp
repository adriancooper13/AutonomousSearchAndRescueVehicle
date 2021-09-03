#include "image-processing.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessing>());
    rclcpp::shutdown();
    return 0;
}