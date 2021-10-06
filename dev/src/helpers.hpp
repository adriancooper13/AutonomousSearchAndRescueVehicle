#include <cmath>
#include <map>
#include "geometry_msgs/msg/quaternion.hpp"

std::map<char, double> euler_from_quaternion(geometry_msgs::msg::Quaternion orientation)
{
    auto rpy = std::map<char, double>();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (orientation.w * orientation.x + orientation.y * orientation.z);
    double cosr_cosp = 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y);
    rpy['x'] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (orientation.w * orientation.y - orientation.z * orientation.x);
    if (std::abs(sinp) >= 1)
        rpy['y'] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        rpy['y'] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
    double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
    rpy['z'] = std::atan2(siny_cosp, cosy_cosp);

    return rpy;
}