/**
 * @file
 * @brief Example node to publish point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <algorithm>
#include <chrono>
#include <memory>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using Cloud = ouster_ros::Cloud;
using Point = ouster_ros::Point;
namespace sensor = ouster::sensor;
auto AngularToQuaternion(double ax, double ay, double az)
  {
      double x,y,z,w;
      double angle = sqrt(ax*ax + ay*ay + az*az);

      if(angle > 0.0){
        x = ax*sin(angle/2.0)/angle;
        y = ay*sin(angle/2.0)/angle;
        z = az*sin(angle/2.0)/angle;
        w = cos(angle/2.0);
      }
      else{
          x=y=z=0.0;
          w=1.0;
      }
        printf("x,y,z,w = [%lf, %lf, %lf, %lf]\n", x,y,z,w);
      return tf::Quaternion(x,y,z,w);
  }
int main(int argc, char** argv) {
    ros::init(argc, argv, "os_cloud_node");
    ros::NodeHandle nh("~");

    auto tf_prefix = nh.param("tf_prefix", std::string{});
    if (!tf_prefix.empty() && tf_prefix.back() != '/') tf_prefix.append("/");
    auto sensor_frame = tf_prefix + "os_sensor";
    auto imu_frame = tf_prefix + "os_imu";
    auto lidar_frame = tf_prefix + "os_lidar";
    auto main_frame = tf_prefix + "odom";

    ouster_ros::OSConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling config service failed");
        return EXIT_FAILURE;
    }

    auto info = sensor::parse_metadata(cfg.response.metadata);
    uint32_t H = info.format.pixels_per_column;
    uint32_t W = info.format.columns_per_frame;

    auto pf = sensor::get_format(info);

    auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    auto xyz_lut = ouster::make_xyz_lut(info);

    Cloud cloud{W, H};
    ouster::LidarScan ls{W, H};

    ouster::ScanBatcher batch(W, pf);

    auto lidar_handler = [&](const PacketMsg& pm) mutable {
        if (batch(pm.buf.data(), ls)) {
            auto h = std::find_if(
                ls.headers.begin(), ls.headers.end(), [](const auto& h) {
                    return h.timestamp != std::chrono::nanoseconds{0};
                });
            if (h != ls.headers.end()) {
                scan_to_cloud(xyz_lut, h->timestamp, ls, cloud);
                lidar_pub.publish(ouster_ros::cloud_to_cloud_msg(
                    cloud, h->timestamp, main_frame));
            }
        }
    };
  tf::TransformBroadcaster broadcaster;
  double testx =0.0;
  double ax= 0.0,ay= 0.0,az = 0.0;
    auto imu_handler = [&](const PacketMsg& p) {
        imu_pub.publish(ouster_ros::packet_to_imu_msg(p, imu_frame, pf));
        ax = ouster_ros::packet_to_imu_msg(p, imu_frame, pf).angular_velocity.x;
        ay = ouster_ros::packet_to_imu_msg(p, imu_frame, pf).angular_velocity.y;
        az = ouster_ros::packet_to_imu_msg(p, imu_frame, pf).angular_velocity.z;
        double x = ouster_ros::packet_to_imu_msg(p, imu_frame, pf).linear_acceleration.x;
        double y = ouster_ros::packet_to_imu_msg(p, imu_frame, pf).linear_acceleration.y;
        // double z = ouster_ros::packet_to_imu_msg(p, imu_frame, pf).linear_acceleration.z;
        // double x_ = atan(y/sqrt(x*x+z*z));
        // double y_ = atan(x/sqrt(y*y+z*z));
        broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(-y/20, x/20, 0, 1), tf::Vector3(0, 0, 0)),
        //tf::Transform(AngularToQuaternion(ax,ay,az), tf::Vector3(0, 0, 0)),
        ouster_ros::packet_to_imu_msg(p, imu_frame, pf).header.stamp,main_frame, sensor_frame));
        testx += 0.001;
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);

    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};
    
    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.imu_to_sensor_transform, main_frame, sensor_frame));
    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.imu_to_sensor_transform, sensor_frame, imu_frame));
    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.lidar_to_sensor_transform, sensor_frame, lidar_frame));
    
   
    ros::spin();

    return EXIT_SUCCESS;
}
