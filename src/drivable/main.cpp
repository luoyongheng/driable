#include <iostream>

#include "CameraModel.h"
#include "ImageManage.h"
#include "Map.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
int main(int argc, char** argv) {
    Camera camera = Camera::GetInstance("/home/lyh/CLionProjects/catkin_ws/src/drivable/front_main.yaml");
    ImageManage imageManage;
    imageManage.LoadImage();
    imageManage.ReProject(camera.GetIntrisic(),camera.GetCameraHeight());
    Map map;
    map.vecImage.push_back(imageManage);
    //map.MapShow(camera.GetIntrisic());

    ros::init(argc, argv, "Point_Cloud");
    ros::NodeHandle n;
    imageManage.cloudPub = n.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    imageManage.odometry = n.advertise<nav_msgs::Odometry>("odom",1);

    //定义模拟激光雷达到base_link之间的坐标变换
    tf::TransformBroadcaster br;
    tf::Transform transform;
    //TODO 这里先假设激光雷达与车体坐标系重合
    transform.setOrigin( tf::Vector3(0, 0, 0) );
    transform.setRotation( tf::Quaternion(0,0,0,1) );
    ros::Time tNow = ros::Time::now();

    //TODO 先假设odom与map坐标系重合，定义里程计与baselink的坐标变换，并发布里程计信息


    //TODO 发布里程计信息给gmapping
    imageManage.BuildOdometry(tNow);
    imageManage.BuildPclPointCloud(tNow);
    ros::Rate r(1.0);
    while(n.ok()){
        br.sendTransform(tf::StampedTransform(transform, tNow, "base_link", "scan"));
        imageManage.PubRosPointCloud();
        r.sleep();
    }
    return 0;
}
