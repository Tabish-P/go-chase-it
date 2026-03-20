#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

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
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int left_bound = img.width / 3;
    int right_bound = 2 * img.width / 3;

    for (int i = 0; i < img.height * img.width * img.step; i += 3)
    {
        if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel)
        {
            int pixel_index = i / 3;
            int column_index = pixel_index % img.width;

            if (column_index < left_bound)
            {
                drive_robot(0.5, 0.5); // Move left
            }
            else if (column_index < right_bound)
            {
                drive_robot(0.5, 0.0); // Move forward
            }
            else
            {
                drive_robot(0.5, -0.5); // Move right
            }
            return; // Exit after finding the first white pixel
        }
    }
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