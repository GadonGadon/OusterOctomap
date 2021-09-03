#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>

#include <cassert>
#include <chrono>
#include <string>
#include <vector>

#include "ouster/types.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>

void callback(const sensor_msgs::ImuPtr& input)
{
  double roll, pitch, yaw;
  double ax = input->angular_velocity.x;
  double ay = input->angular_velocity.y;
  double az = input->angular_velocity.z;
  printf("ax, ay, az : [%lf, %lf, %lf]\n", ax, ay, az);

  roll = 180*atan(ay/sqrt(ax*ax+az*az))/PI;
  pitch = 180*atan(ax/sqrt(ay*ay+az*az))/PI;
  printf("roll, pitch : [%lf, %lf]\n", roll, pitch);
}
int main(int argc, char** argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Subscriber imuSub = n.subscribe("/os_cloud_node/imu", 100, callback);
    ros::Rate r(1000);
    while(n.ok()){
 
        printf("123123\n");  
        r.sleep();
        
        //ros::spin();

    }
    
}