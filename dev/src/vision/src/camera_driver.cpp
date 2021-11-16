#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#define DEBUG false

class CameraDriver : public rclcpp::Node
{
    private:
        cv::VideoCapture capture;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
        bool video0;

    public:
        CameraDriver() : Node("camera_driver")
        {
            image_publisher = create_publisher<sensor_msgs::msg::Image>(
                "image_raw",
                10
            );

            using namespace std::chrono_literals;
            capture = cv::VideoCapture("/dev/video0", cv::CAP_V4L);
            video0 = true;

            timer = create_wall_timer(
                0.01s,
                std::bind(&CameraDriver::read_image, this)
            );

	    RCLCPP_INFO(get_logger(), "%s node has started", get_name());
        }

        ~CameraDriver()
        {
            capture.release();
        }

    private:
        void try_toggle_input()
        {
            if (video0)
            {
                capture = cv::VideoCapture("/dev/video1", cv::CAP_V4L);
                RCLCPP_INFO(get_logger(), "Connected to /dev/video1");
                video0 = false;
            }
            else
            {
                capture = cv::VideoCapture("/dev/video0", cv::CAP_V4L);
                RCLCPP_INFO(get_logger(), "Connected to /dev/video0");
                video0 = true;
            }
        }
        
        void read_image()
        {
            cv_bridge::CvImage image;
            image.encoding = sensor_msgs::image_encodings::BGR8;
            image.header = std_msgs::msg::Header();
            image.header.stamp = now();

            if (!capture.read(image.image))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Could not read image"
                );
		        try_toggle_input();
		        return;
            }

            auto message = image.toImageMsg();
            image_publisher->publish(*message);

            if (DEBUG)
            {
                show_image(image.image);
            }
        }

        void show_image(cv::Mat image)
        {
            cv::imshow("image", image);
            cv::waitKey(1);
        }
};  

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraDriver>());
    rclcpp::shutdown();
    return 0;
}
