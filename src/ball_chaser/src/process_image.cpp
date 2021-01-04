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
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if(!client.call(srv))
    {
        ROS_ERROR("ERROR! Could not call the DriveToTarget service from within the 'drive_robot' function in ball_chaser package, 'process_image.cpp' file.");
    }
}




// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int rows = img.height; // 800
    int cols = img.step;   // 2400
    int n = rows * cols;   // 1920000

    int num_white_pixels_found = 0;
    int x_bar_sum = 0;
    int x_bar = -1;
    float m = -2.0 / (float)cols;

    float lin_x = 0;
    float ang_z = 0;


    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    for(int i=0; i<rows; i++) // 800 rows  
    {
        for(int j=0; j<cols; j+=3) // 2400 columns, by 3's (800 effective collumns)
        {
            int index = (i * cols) + j;
            if(index < 0 or index + 2 >= n)
            {
                ROS_ERROR("ERROR! Index out of rangefrom within the 'drive_robot' function in ball_chaser package, 'process_image.cpp' file.");
                return;
            }

            int r = (int)img.data[index];   // untested (indexing is wrong...)
            int g = (int)img.data[index+1]; // untested
            int b = (int)img.data[index+2]; // untested

            if(r == 255 and g == 255 and b == 255)
            {
                // save x_bar to find horizontal centroid of the white ball
                x_bar_sum += j;
                ++num_white_pixels_found;
            }
        }    
    }

    // calculate x_bar
    if(x_bar_sum <= 0 or num_white_pixels_found <= 0)
    {
        x_bar = -1;  // null value for area centroid (white ball was NOT in image frame)
        lin_x = 0.0; // halt forward movement
        ang_z = 0.0; // halt rotational movement
    }
    else
    {
        x_bar = x_bar_sum / num_white_pixels_found; // area centroid (white ball was found in image frame)
        // transform from [0, 2400] to [1, -1]: ang_z = (-2.0 / 2400.0) * x_bar + 1.0
        lin_x = 0.3;
        ang_z = m * (float)x_bar + 1.0;
    }

    drive_robot(lin_x, ang_z);


    // ROS_INFO_STREAM(lin_x);
    // ROS_INFO_STREAM(ang_z);
    // ROS_INFO_STREAM("---");

    


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
    
    //S_INFO("---------");

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
