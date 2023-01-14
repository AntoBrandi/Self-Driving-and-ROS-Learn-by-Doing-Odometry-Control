#include "bumperbot_examples/tf_examples.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_examples");
    ros::NodeHandle nh;
    TfExamples tfExamples(nh);
    ros::spin();

    return 0;
}