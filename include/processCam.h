//
// Created by unicorn on 2020/9/22.
//

#ifndef LASERCAMERACALIBRATE_PROCESSCAM_H
#define LASERCAMERACALIBRATE_PROCESSCAM_H

#include <PinholeCamera.h>
#include "calcCamPose.h"
#include <memory>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
class CameraUtil{
private:
    shared_ptr<CamPoseEst> poseEst;
    PinholeCameraPtr camera;
    std::vector<cv::Point3f> p3ds;
    std::vector<cv::Point2f> p2ds;
    Eigen::Matrix4d Twc;
    cv::Mat img;
public:
    CameraUtil(const std::string &cameraName, int imageWidth,
        int imageHeight, double k1, double k2, double p1,
        double p2, double fx, double fy, double cx,
        double cy, CalibrBoardInfo info){
        camera=boost::shared_ptr<PinholeCamera>(new PinholeCamera(cameraName,imageWidth,imageHeight,k1,k2,p1,p2,fx,fy,cx,cy));
        poseEst=make_shared<CamPoseEst>(CamPoseEst(info));
    }
    void ProcessImg(const sensor_msgs::ImageConstPtr& msgRGB){
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception& e)
        {
            printf("cv_bridge exception: %s", e.what());
            return;
        }
        img=cv_ptrRGB->image;
        poseEst->FindTargetCorner(img,p3ds,p2ds);
        poseEst->EstimatePose(p3ds,p2ds,camera,Twc,img);
    }
    Eigen::Matrix4d GetPose(){return Twc;}
    cv::Mat GetImg(){return img.clone();}
};
#endif //LASERCAMERACALIBRATE_PROCESSCAM_H
