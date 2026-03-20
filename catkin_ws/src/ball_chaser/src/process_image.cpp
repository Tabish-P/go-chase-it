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
void process_image_callback(const sensor_msgs::ImageConstPtr& img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    drive_robot(0.0, 0.0); // stop the robot if no white ball is found
    
    for (size_t i = 0; i < img->height * img->step; i+=3) {
        int red = img->data[i];
        int green = img->data[i + 1];
        int blue = img->data[i + 2];

        // check if you found a white pixel
        if (red == white_pixel && green == white_pixel && blue == white_pixel) {
            int pixel_position = i % img->step; // get the horizontal position of the pixel
            if (pixel_position < img->step / 3) {
                // left side
                drive_robot(0.5, 0.5); // move forward and turn left
            } else if (pixel_position < 2 * img->step / 3) {
                // middle
                drive_robot(0.5, 0.0); // move forward
            } else {
                // right side
                drive_robot(0.5, -0.5); // move forward and turn right
            }
            return; // exit the function after finding the first white pixel
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