//
// Created by unicorn on 2020/9/22.
//

#ifndef LASERCAMERACALIBRATE_CALIBRATE_H
#define LASERCAMERACALIBRATE_CALIBRATE_H

#include "processScan.h"
#include "processCam.h"

class Calibrate{
public:
    shared_ptr<LaserUtil> laser;
    shared_ptr<CameraUtil> cam;
    Calibrate(bool vis,CalibrBoardInfo info,int w,int h,cv::Mat K,cv::Mat dist){
        laser=make_shared<LaserUtil>(LaserUtil(vis));
        cam=make_shared<CameraUtil>(
                CameraUtil("camera",w,
                        h,dist.at<double>(0,0),dist.at<double>(1,0),dist.at<double>(2,0),
                                dist.at<double>(3,0),dist.at<double>(0,0),K.at<double>(1,1),K.at<double>(0,2),
                                        K.at<double>(1,2),info)
                );
    }

    void ProcessData(const sensor_msgs::ImagePtr &img,sensor_msgs::LaserScanConstPtr scan){
        cam->ProcessImg(img);
        laser->LaserCallBack(scan);
    }

};
#endif //LASERCAMERACALIBRATE_CALIBRATE_H
