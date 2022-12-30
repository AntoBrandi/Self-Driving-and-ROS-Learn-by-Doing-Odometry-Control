#include <ros/ros.h>
#include "bumperbot_examples/AddTwoInts.h"


int main(int argc, char **argv)
{
    // When this script is launched, get the args that are passed
    // to the script when is launched. This script requires 2 args from the user
    // that are the two numbers that will be added
    ros::init(argc, argv, "simple_service_client_cpp");
    if (argc != 3)
    {
        ROS_INFO("Requested two arguments");
        return 1;
    }


    // Sum the two numbers passed as arguments to this script 
    // by calling the action server that adds two integers
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<bumperbot_examples::AddTwoInts>("add_two_ints");
    bumperbot_examples::AddTwoInts srv;
    srv.request.a = atoi(argv[1]);
    srv.request.b = atoi(argv[2]);

    // Show the response of the service
    ROS_INFO("Requesting %d + %d", (int)srv.request.a, (int)srv.request.b);
    if (client.call(srv))
    {
        ROS_INFO("Service Response %d", (int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }

    return 0;
}
