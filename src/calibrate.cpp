//
// Created by unicorn on 2020/9/21.
//
#include <ros/ros.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include "processScan.h"
using namespace std;

int main(int argc,char ** argv){

    ros::init(argc,argv,"calib");
    ros::NodeHandle nh;

    LaserUtil laserUtil(nh);

    string image_topic="/camera/color/image_raw",scan_topic="/scan";
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, image_topic, 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, scan_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,scan_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    ros::spin();
}