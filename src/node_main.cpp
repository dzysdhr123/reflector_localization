#include <glog/logging.h>

#include "ros_node.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reflector_localization");
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    LOG(INFO) << "Start Reflector EKF SLAM";
    Reflector_localization::Node node;
    ros::spin();
    google::ShutdownGoogleLogging();

    return 0;
}
