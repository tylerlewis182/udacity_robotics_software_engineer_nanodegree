#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <string>
#include <typeinfo>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot






}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    for(int i=0; i<img.height * img.step; i+=3)
    {
        int r = (int)img.data[i];   // untested 
        int g = (int)img.data[i+1]; // untested
        int b = (int)img.data[i+2]; // untested
    }

    /*
        Image Coordinates:

        ----- +x
        |
        |
        +y

    */


    //ROS_INFO("---------");
    //ROS_INFO_STREAM(typeid(img.data).name());     // St6vectorIhSaIhEE (aka: vector<unsigned char>)
    //ROS_INFO_STREAM(typeid(img.data[0]).name());  // h (aka: unsigned char, so cast to int before comparison with 'white_pixel')
    //ROS_INFO_STREAM(img.data.size());             // 1920000
    //ROS_INFO_STREAM(img.height);                  // 800
    //ROS_INFO_STREAM(img.width);                   // 800
    //ROS_INFO_STREAM(img.step);                    // 2400
    //ROS_INFO_STREAM((int)img.data[1919998]);      // 155 (when white ball is NOT in bottom right corner of image)
    //ROS_INFO_STREAM((int)img.data[1919998]);      // 255 (when white ball IS in bottom right corner of image)
    
    // // when the blue ball covers the top left pixel:
    // ROS_INFO_STREAM((int)img.data[0]);           // 0   (red)
    // ROS_INFO_STREAM((int)img.data[1]);           // 0   (blue)
    // ROS_INFO_STREAM((int)img.data[2]);           // 255 (green)
    
    //ROS_INFO("---------");

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
