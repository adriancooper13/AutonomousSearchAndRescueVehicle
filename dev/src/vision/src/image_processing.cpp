#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#define WIDTH 360
#define HEIGHT 240
#define DEBUG true

const auto RED = cv::Scalar(0, 0, 255);
const auto GREEN = cv::Scalar(0, 255, 0);
const auto BLUE = cv::Scalar(255, 0, 0);

using namespace std::chrono_literals;

class ImageProcessing : public rclcpp::Node
{
    private:
        int MiddlePos;
        std::vector<int> histogramLane;
        cv::Mat frame, framePers, frameFinal, frameFinalDuplicate;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr direction_publisher;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
        rclcpp::TimerBase::SharedPtr timer;

    public:
        ImageProcessing() : Node("image_processing")
        {
            direction_publisher = create_publisher<std_msgs::msg::Int32>("direction", 10);
            image_subscription = create_subscription<sensor_msgs::msg::Image>(
                "camera/image_raw",
                10,
                std::bind(&ImageProcessing::process_image, this, std::placeholders::_1)
            );

            if (DEBUG)
            {
                timer = create_wall_timer(1ms, std::bind(&ImageProcessing::image_show, this));
            }

            RCLCPP_INFO(get_logger(), "%s node has started.", get_name());
        }

    private:
        void process_image(const sensor_msgs::msg::Image::SharedPtr message)
        {
            frame = cv_bridge::toCvCopy(message, message->encoding)->image;

            Perspective();
            Threshold();
            Histogram();
            BallFinder();
            LaneCenter();
        }

        void Perspective()
        {
            // create a frame of reference... adjust these as needed. They represent the 4 corners of the box.
            cv::Point2f Source[] = {
                cv::Point2f(30, HEIGHT / 2),
                cv::Point2f(WIDTH - 30, HEIGHT / 2),
                cv::Point2f(0, HEIGHT),
                cv::Point2f(WIDTH, HEIGHT)
            };
            // Point2f Destination[] = {Point2f(25, 150), Point2f(330, 150), Point2f(0, 210), Point2f(360, 210)};
            cv::Point2f Destination[] = {
                cv::Point2f(80, 0),
                cv::Point2f(280, 0),
                cv::Point2f(80, 240),
                cv::Point2f(280, 240)
            };
            // this is to join the 4 points via openCV
            int lineWidth = 2;
            
            line(frame, Source[0], Source[1], RED, lineWidth); // goes from top left to top right
            line(frame, Source[1], Source[3], RED, lineWidth); // goes from top right to bottom right
            line(frame, Source[3], Source[2], RED, lineWidth); // goes from bottom right to bottom left
            line(frame, Source[2], Source[0], RED, lineWidth); // goes from bottom left to top left
            
            auto matrix = getPerspectiveTransform(Source, Destination);
            warpPerspective(frame, framePers, matrix, cv::Size(WIDTH, HEIGHT));
        }

        void Threshold()
        {
            cv::Mat frameThresh, frameEdge, frameGray;
            cvtColor(framePers, frameGray, cv::COLOR_RGB2GRAY);
            // frame input name, min threshold for white, max threshold for white, frame output name. Tweak these as necessary, but min threshold may want to go down if indoors.
            // find the white in the image.
            inRange(frameGray, 220, 255, frameThresh); // 137 looked good indoors at night, 165 looked good indoors during the day
            // input, output, minimum threshold for histerisis process. always goes 100 for minimum. 2nd threshhold, usually go 500, axa matrix, advanced gradient?
            // edge detection         
            Canny(frameGray, frameEdge, 400, 600, 3, false); // was 250, 600
            // merge our images together into final frame
            add(frameThresh, frameThresh, frameFinal);
            cvtColor(frameFinal, frameFinal, cv::COLOR_GRAY2RGB);
            // used in histogram function only.
            cvtColor(frameFinal, frameFinalDuplicate, cv::COLOR_RGB2BGR);
        }

        void Histogram()
        {
            // resize to the size of the lane.
            histogramLane.resize(frame.size().width);
            histogramLane.clear();
            
            cv::Mat ROILane;
            for (int i = 0; i < frame.size().width; i++)
            {
                // reason of interest strip
                ROILane = frameFinalDuplicate(cv::Rect(i, 140, 1, 100));
                divide(255, ROILane, ROILane);
                histogramLane.push_back((int)(sum(ROILane)[0]));
            }
        }

        void LaneCenter()
        {
            int frame_center = WIDTH / 2;
            
            line(frameFinal, cv::Point2f(frame_center, 0), cv::Point2f(frame_center, HEIGHT), BLUE, 3);
            
            // difference between true center and center ball...
            int result = MiddlePos - frame_center;

            // Publish result
            auto message = std_msgs::msg::Int32();
            message.data = result;
            direction_publisher->publish(message);
        }

        void BallFinder()
        {
            // iterator to point to max intensity spot
            std::vector<int>::iterator LeftPtr;
            // scans from left-most pixel to middle pixel
            LeftPtr = max_element(histogramLane.begin(), histogramLane.begin() + 120);
            auto LeftLanePos = distance(histogramLane.begin(), LeftPtr);
            
            // iterator to point to max intensity spot
            std::vector<int>::iterator RightPtr;
            // scans from left-most pixel to middle pixel
            RightPtr = max_element(histogramLane.end() - 119, histogramLane.end());
            auto RightLanePos = distance(histogramLane.begin(), RightPtr);
            
            
            std::vector<int>::iterator MiddlePtr;
            MiddlePtr = max_element(histogramLane.begin() + 121, histogramLane.end() - 120);
            MiddlePos = distance(histogramLane.begin(), MiddlePtr);
            
            // middle is at pixel column 180
            int midDist = abs(180 - MiddlePos);
            int leftDist = abs(180 - LeftLanePos);
            int rightDist = abs(180 - RightLanePos);
            
            if (midDist <= leftDist && midDist <= rightDist) {
                MiddlePos = MiddlePos;
            } else if (leftDist <= midDist && leftDist <= rightDist) {
                MiddlePos = LeftLanePos;
            } else {
                MiddlePos = RightLanePos;
            }
            
            line(frameFinal, cv::Point2f(MiddlePos, 0), cv::Point2f(MiddlePos, 240), GREEN, 2);
        }

        void image_show()
        {
            if (!frame.empty())
            {
                cv::imshow("raw_image", frame);
                cv::waitKey(1);
            }
            
            if (!framePers.empty())
            {
                cv::imshow("perspective_image", framePers);
                cv::waitKey(1);
            }

            if (!frameFinal.empty())
            {
                cv::imshow("final_image", frameFinal);
                cv::waitKey(1);
            }

            if (!frameFinalDuplicate.empty())
            {
                cv::imshow("final_dup_image", frameFinalDuplicate);
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