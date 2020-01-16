//
// Created by lyh on 2020/1/6.
//

#ifndef DRIVABLE_IMAGEMANAGE_H
#define DRIVABLE_IMAGEMANAGE_H

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pclPointCloudXYZ;
typedef boost::shared_ptr<pclPointCloudXYZ> pclPointCloudXYZPtr;

class ImageManage{
public:
    void ReProject(const Eigen::Matrix3d& K,const double& camH){
        for(auto ptsImage : vecPtsImage){
            double ZSpace = camH * K(1,1)/(ptsImage.y - K(1,2));
            assert(ZSpace > 0);
            double XSpace = ZSpace * (ptsImage.x - K(0,2))/K(0,0);
            cv::Point3d ptsSpace(XSpace,camH,ZSpace);
            vecPtsSpace.push_back(ptsSpace);
        }
    }

    void LoadImage(const string& imageFile = ""){
        if(imageFile.empty()){
            cv::Mat image(948,1828,CV_8UC1,cv::Scalar(255,255,255,1));
            //TODO 图像矫正
            cv::line(image,cv::Point2d(0,947),cv::Point2d(0,810),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(0,810),cv::Point2d(150,810),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(150,810),cv::Point2d(150,735),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(150,735),cv::Point2d(300,735),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(300,735),cv::Point2d(300,660),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(300,660),cv::Point2d(450,660),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(450,660),cv::Point2d(450,780),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(450,780),cv::Point2d(570,760),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(570,760),cv::Point2d(570,660),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(570,660),cv::Point2d(660,660),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(660,660),cv::Point2d(660,690),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(660,690),cv::Point2d(780,690),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(780,690),cv::Point2d(780,630),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(780,630),cv::Point2d(810,630),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(810,630),cv::Point2d(810,650),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(810,650),cv::Point2d(870,650),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(870,650),cv::Point2d(890,610),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(890,610),cv::Point2d(950,610),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(950,610),cv::Point2d(950,590),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(950,590),cv::Point2d(1200,590),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(1200,590),cv::Point2d(1260,620),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(1260,620),cv::Point2d(1350,620),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(1350,620),cv::Point2d(1370,670),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(1370,670),cv::Point2d(1520,670),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(1520,670),cv::Point2d(1540,720),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(1540,720),cv::Point2d(1660,720),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(1660,720),cv::Point2d(1690,770),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(1690,770),cv::Point2d(1690,870),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(1690,870),cv::Point2d(1827,870),cv::Scalar(0,0,0));
            cv::line(image,cv::Point2d(1827,870),cv::Point2d(1827,947),cv::Scalar(0,0,0));
            //cv::imshow("test.png",image);
            cv::waitKey(0);
        } else {
            cv::Mat image = cv::imread(imageFile);
            //cv::cvtColor()
            //TODO 图像矫正
        }
    }

    ImageManage(const vector<cv::Point2d>& p){
        vecPtsImage.clear();
        pclCloud = boost::make_shared<pclPointCloudXYZ>();
        vecPtsImage = p;
    }
    ImageManage(){
        pclCloud = boost::make_shared<pclPointCloudXYZ>();
        vecPtsImage.push_back(cv::Point2d(0,947));
        vecPtsImage.push_back(cv::Point2d(0,810));
        vecPtsImage.push_back(cv::Point2d(150,810));
        vecPtsImage.push_back(cv::Point2d(150,735));
        vecPtsImage.push_back(cv::Point2d(300,735));
        vecPtsImage.push_back(cv::Point2d(300,660));
        vecPtsImage.push_back(cv::Point2d(450,660));
        vecPtsImage.push_back(cv::Point2d(450,780));
        vecPtsImage.push_back(cv::Point2d(570,760));
        vecPtsImage.push_back(cv::Point2d(570,660));
        vecPtsImage.push_back(cv::Point2d(660,660));
        vecPtsImage.push_back(cv::Point2d(660,690));
        vecPtsImage.push_back(cv::Point2d(780,690));
        vecPtsImage.push_back(cv::Point2d(780,630));
        vecPtsImage.push_back(cv::Point2d(810,630));
        vecPtsImage.push_back(cv::Point2d(810,650));
        vecPtsImage.push_back(cv::Point2d(870,650));
        vecPtsImage.push_back(cv::Point2d(890,610));
        vecPtsImage.push_back(cv::Point2d(950,610));
        vecPtsImage.push_back(cv::Point2d(950,590));
        vecPtsImage.push_back(cv::Point2d(1200,590));
        vecPtsImage.push_back(cv::Point2d(1260,620));
        vecPtsImage.push_back(cv::Point2d(1350,620));
        vecPtsImage.push_back(cv::Point2d(1370,670));
        vecPtsImage.push_back(cv::Point2d(1520,670));
        vecPtsImage.push_back(cv::Point2d(1540,720));
        vecPtsImage.push_back(cv::Point2d(1660,720));
        vecPtsImage.push_back(cv::Point2d(1690,770));
        vecPtsImage.push_back(cv::Point2d(1690,870));
        vecPtsImage.push_back(cv::Point2d(1827,870));
        vecPtsImage.push_back(cv::Point2d(1827,947));
    }

    vector<cv::Point3d> GetPointsSpace(){
        return vecPtsSpace;
    }

    void BuildPclPointCloud(const ros::Time& t)
    {
        pcl::PointXYZ target_pt;
        for(auto& pSpace : vecPtsSpace){
            target_pt.x =  pSpace.x;
            target_pt.y =  pSpace.z;
            target_pt.z =  pSpace.y;
            pclCloud->points.push_back( target_pt );
        }
        pclCloud->width = pclCloud->points.size();
        pclCloud->header.stamp = pcl_conversions::toPCL( ros::Time::now());
        pclCloud->height = 1;
        pcl::toROSMsg( *pclCloud, ROSCloud );
        ROSCloud.header.frame_id = "scan";
        ROSCloud.header.stamp = t;
    }

    void BuildOdometry(const ros::Time& t)
    {
        //先假设机器人不动试试
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);//没有旋转
        odom_trans.header.stamp = t;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = 0;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;

        odom.header.stamp = t;
        odom.header.frame_id = "odom";
        //set the position
        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;
    }

    void PubRosPointCloud()
    {
        static tf::TransformBroadcaster odom_broadcaster;
        cloudPub.publish(ROSCloud);
        odom_broadcaster.sendTransform(odom_trans);
        odometry.publish(odom);
    }
    ros::Publisher cloudPub;
    ros::Publisher odometry;
private:
    vector<cv::Point2d> vecPtsImage;
    vector<cv::Point3d> vecPtsSpace;
    nav_msgs::Odometry odom;
    pclPointCloudXYZPtr pclCloud;
    sensor_msgs::PointCloud2  ROSCloud;
    geometry_msgs::TransformStamped odom_trans;
};
#endif //DRIVABLE_IMAGEMANAGE_H
