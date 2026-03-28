#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <cmath>
#include <algorithm>

// Define a global client that can request services
ros::ServiceClient client;

namespace {
constexpr int kWhiteThreshold = 200;
constexpr float kMaxLinearSpeed = 0.6F;
constexpr float kMinLinearSpeed = 0.12F;
constexpr float kMaxAngularSpeed = 0.8F;
constexpr float kTurnGain = 1.2F;
constexpr float kCommandEpsilon = 0.01F;

float last_linear_x = 0.0F;
float last_angular_z = 0.0F;

float clamp_value(float value, float minimum, float maximum)
{
    return std::max(minimum, std::min(maximum, value));
}

void drive_robot_if_changed(float lin_x, float ang_z)
{
    const bool linear_changed = std::fabs(lin_x - last_linear_x) > kCommandEpsilon;
    const bool angular_changed = std::fabs(ang_z - last_angular_z) > kCommandEpsilon;

    if (linear_changed || angular_changed) {
        drive_robot(lin_x, ang_z);
        last_linear_x = lin_x;
        last_angular_z = ang_z;
    }
}
}  // namespace

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::ImageConstPtr& img)
{
    if (img->width == 0 || img->height == 0 || img->step < img->width * 3) {
        drive_robot_if_changed(0.0F, 0.0F);
        return;
    }

    // Use centroid of bright pixels for stable steering instead of first-pixel detection.
    std::size_t white_pixel_count = 0;
    double white_pixel_column_sum = 0.0;

    for (std::size_t row = 0; row < img->height; ++row) {
        const std::size_t row_start = row * img->step;
        for (std::size_t col = 0; col < img->width; ++col) {
            const std::size_t pixel_index = row_start + (col * 3);
            const int red = img->data[pixel_index];
            const int green = img->data[pixel_index + 1];
            const int blue = img->data[pixel_index + 2];

            if (red >= kWhiteThreshold && green >= kWhiteThreshold && blue >= kWhiteThreshold) {
                ++white_pixel_count;
                white_pixel_column_sum += static_cast<double>(col);
            }
        }
    }

    if (white_pixel_count == 0) {
        drive_robot_if_changed(0.0F, 0.0F);
        return;
    }

    const double image_center = static_cast<double>(img->width) / 2.0;
    const double centroid_col = white_pixel_column_sum / static_cast<double>(white_pixel_count);
    const double normalized_error = (image_center - centroid_col) / image_center;

    float angular_z = static_cast<float>(kTurnGain * normalized_error);
    angular_z = clamp_value(angular_z, -kMaxAngularSpeed, kMaxAngularSpeed);

    const double visibility_ratio = static_cast<double>(white_pixel_count) /
        static_cast<double>(img->width * img->height);

    float linear_x = static_cast<float>(kMaxLinearSpeed - (visibility_ratio * 2.5));
    linear_x = clamp_value(linear_x, kMinLinearSpeed, kMaxLinearSpeed);

    // Slow forward motion when turn error is high to prevent overshoot.
    const float turn_penalty = 1.0F - (0.6F * clamp_value(static_cast<float>(std::fabs(normalized_error)), 0.0F, 1.0F));
    linear_x *= turn_penalty;

    drive_robot_if_changed(linear_x, angular_z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}