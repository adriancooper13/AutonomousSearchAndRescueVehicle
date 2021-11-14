#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "custom_interfaces/msg/threshold_adjustment.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"

#define WIDTH 640
#define HEIGHT 480
#define DEBUG false

const auto RED = cv::Scalar(0, 0, 255);
const auto GREEN = cv::Scalar(0, 255, 0);
const auto BLUE = cv::Scalar(255, 0, 0);

class ImageProcessing : public rclcpp::Node
{
    private:
        int middle_pos, lower_threshold, upper_threshold;
        std::vector<int> histogram_lane;
        cv::Mat frame, frame_perspective, frame_final;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr direction_publisher;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
        rclcpp::Subscription<custom_interfaces::msg::ThresholdAdjustment>::SharedPtr threshold_subscription;
        rclcpp::TimerBase::SharedPtr timer;

    public:
        ImageProcessing() : Node("image_processing")
        {
            lower_threshold = 100;
            upper_threshold = 255;

            direction_publisher = create_publisher<std_msgs::msg::Int32>("direction", 10);
            image_subscription = create_subscription<sensor_msgs::msg::Image>(
                "image_raw",
                10,
                std::bind(&ImageProcessing::process_image, this, std::placeholders::_1)
            );
            threshold_subscription = create_subscription<custom_interfaces::msg::ThresholdAdjustment>(
                "vision_threshold_adjustment",
                10,
                std::bind(&ImageProcessing::adjust_thresholds, this, std::placeholders::_1)
            );

            if (DEBUG)
            {
                using namespace std::chrono_literals;
                timer = create_wall_timer(1ms, std::bind(&ImageProcessing::image_show, this));
            }

            RCLCPP_INFO(get_logger(), "%s node has started.", get_name());
        }

    private:
        void process_image(const sensor_msgs::msg::Image::SharedPtr message)
        {
            frame = cv_bridge::toCvCopy(message, message->encoding)->image;

            perspective();
            threshold();
            histogram();
            find_largest_ball();
            lane_center();
        }

        void perspective()
        {
            // create a frame of reference... adjust these as needed. They represent the 4 corners of the box.
            cv::Point2f source[] = {
                cv::Point2f(30, HEIGHT / 2),
                cv::Point2f(WIDTH - 30, HEIGHT / 2),
                cv::Point2f(0, HEIGHT),
                cv::Point2f(WIDTH, HEIGHT)
            };
            // Point2f Destination[] = {Point2f(25, 150), Point2f(330, 150), Point2f(0, 210), Point2f(360, 210)};
            cv::Point2f destination[] = {
                cv::Point2f(80, 0),
                cv::Point2f(280, 0),
                cv::Point2f(80, 240),
                cv::Point2f(280, 240)
            };
            // this is to join the 4 points via openCV
            int line_width = 2;
            
            line(frame, source[0], source[1], RED, line_width); // goes from top left to top right
            line(frame, source[1], source[3], RED, line_width); // goes from top right to bottom right
            line(frame, source[3], source[2], RED, line_width); // goes from bottom right to bottom left
            line(frame, source[2], source[0], RED, line_width); // goes from bottom left to top left
            
            auto matrix = getPerspectiveTransform(source, destination);
            warpPerspective(frame, frame_perspective, matrix, cv::Size(WIDTH, HEIGHT));
        }

        void threshold()
        {
            cv::Mat frame_thresh, frame_edge, frame_gray;
            cvtColor(frame_perspective, frame_gray, cv::COLOR_BGR2GRAY);
            // frame input name, min threshold for white, max threshold for white, frame output name. Tweak these as necessary, but min threshold may want to go down if indoors.
            // find the white in the image.
            inRange(frame_gray, lower_threshold, upper_threshold, frame_thresh); // 137 looked good indoors at night, 165 looked good indoors during the day
            // input, output, minimum threshold for histerisis process. always goes 100 for minimum. 2nd threshhold, usually go 500, axa matrix, advanced gradient?
            // edge detection         
            Canny(frame_gray, frame_edge, 250, 600, 3, false); // was 250, 600
            // merge our images together into final frame
            add(frame_thresh, frame_edge, frame_final);
            cvtColor(frame_final, frame_final, cv::COLOR_GRAY2RGB);
        }

        void histogram()
        {
            // resize to the size of the lane.
            histogram_lane.resize(frame.size().width);
            histogram_lane.clear();
            
            int top = 20;
            cv::Mat roi_lane, frame_final_bgr;
            cvtColor(frame_final, frame_final_bgr, cv::COLOR_RGB2BGR);
            for (int i = 0; i < frame.size().width; i++)
            {
                // reason of interest strip
                roi_lane = frame_final_bgr(cv::Rect(i, top, 1, HEIGHT - top));
                divide(255, roi_lane, roi_lane);
                histogram_lane.push_back((int)(sum(roi_lane)[0]));
            }
        }

        void lane_center()
        {
            int frame_center = WIDTH / 2;
            
            line(frame_final, cv::Point2f(frame_center, 0), cv::Point2f(frame_center, HEIGHT), BLUE, 3);
            
            // difference between true center and center ball...
            int result = middle_pos - frame_center;

            // Publish result
            auto message = std_msgs::msg::Int32();
            message.data = result;
            direction_publisher->publish(message);
        }

        void find_middle_ball()
        {
            // iterator to point to max intensity spot
            std::vector<int>::iterator left_ptr;
            // scans from left-most pixel to left-middle pixel
            left_ptr = max_element(histogram_lane.begin(), histogram_lane.begin() + 120);
            auto left_lane_pos = distance(histogram_lane.begin(), left_ptr);
            
            // iterator to point to max intensity spot
            std::vector<int>::iterator right_ptr;
            // scans from right-middle pixel to right-most pixel
            right_ptr = max_element(histogram_lane.end() - 119, histogram_lane.end());
            auto right_lane_pos = distance(histogram_lane.begin(), right_ptr);
            
            // scans from left-middle pixel to right-middle pixel
            std::vector<int>::iterator middle_ptr;
            middle_ptr = max_element(histogram_lane.begin() + 121, histogram_lane.end() - 120);
            middle_pos = distance(histogram_lane.begin(), middle_ptr);
            
            // middle is at pixel column 180
            int mid_dist = abs(180 - middle_pos);
            int left_dist = abs(180 - left_lane_pos);
            int right_dist = abs(180 - right_lane_pos);
            
            if (mid_dist <= left_dist && mid_dist <= right_dist) {
                middle_pos = middle_pos;
            } else if (left_dist <= mid_dist && left_dist <= right_dist) {
                middle_pos = left_lane_pos;
            } else {
                middle_pos = right_lane_pos;
            }
            
            line(frame_final, cv::Point2f(middle_pos, 0), cv::Point2f(middle_pos, 240), GREEN, 2);
        }

        void find_largest_ball()
        {
            // iterator to point to max intensity spot
            std::vector<int>::iterator whitest_ptr;
            // scans from left-most pixel to left-middle pixel
            whitest_ptr = max_element(histogram_lane.begin(), histogram_lane.end());
            middle_pos = distance(histogram_lane.begin(), whitest_ptr);
            
            line(frame_final, cv::Point2f(middle_pos, 0), cv::Point2f(middle_pos, 240), GREEN, 2);
        }

        void adjust_thresholds(const custom_interfaces::msg::ThresholdAdjustment::SharedPtr threshold_adjustment)
        {
            int lower_adj = threshold_adjustment->lower_adjustment;
            int upper_adj = threshold_adjustment->upper_adjustment;
            if (lower_threshold + lower_adj >= 0 && lower_threshold + lower_adj <= 255)
                lower_threshold += lower_adj;
            if (upper_threshold + upper_adj >= 0 && upper_threshold + upper_adj <= 255)
                upper_threshold += upper_adj;
        }

        void image_show()
        {
            if (!frame.empty())
            {
                cv::imshow("raw_image", frame);
                cv::waitKey(1);
            }
            
            if (!frame_perspective.empty())
            {
                cv::imshow("perspective_image", frame_perspective);
                cv::waitKey(1);
            }

            if (!frame_final.empty())
            {
                cv::imshow("final_image", frame_final);
                cv::waitKey(1);
            }
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessing>());
    rclcpp::shutdown();
    return 0;
}
