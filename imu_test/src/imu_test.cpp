#include <iostream>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

void imu_callback(const sensor_msgs::ImuPtr& msg)
{
    printf("callback\n");
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_test");
    ros::NodeHandle n("~");

    ros::Subscriber imu_sub = n.subscribe("/os_cloud_node/imu", 100, imu_callback);
    while(n.ok())
    {
        ros::spin();
    }
    return 0;
}