
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "Camera.h"
#include "mutichessboard/ChessboradStruct.h"
#include "mutichessboard/HeaderCB.h"
#include "mutichessboard/CornerDetAC.h"
#include "mutichessboard/corealgmatlab.h"
using namespace std;
enum PatternType
{
    KALIBR_TAG_PATTERN = 1,
    APRIL_TAG_ONE = 2,
    CHESS = 3,
    CIRCLE = 4
};

struct CalibrBoardInfo
{
public:
    CalibrBoardInfo()
    {
        pt_ = CHESS;
        tagSize_ = 0.03;
        rows_ = 9;
        cols_ = 12;
    };

    CalibrBoardInfo(PatternType pt, double tagsize, double tagspacing = 0.3,int rows = 6, int cols = 6):
            pt_(pt),
            tagSize_(tagsize),
            tagSpacing_(tagspacing),
            rows_(rows),
            cols_(cols)
         {};
    PatternType pt_;
    double tagSize_;
    double tagSpacing_;
    int rows_;
    int cols_;
};

class CamPoseEst
{
public:
    CamPoseEst(){};
    CamPoseEst(CalibrBoardInfo info):CalBoardInfo_(info){};

    void FindTargetCorner(cv::Mat &img_raw,
                                      vector<vector<cv::Point2f>> &twochesscorners,
                                      vector<vector<cv::Point3f>> &twochesscorners3d);

    bool EstimatePose(const std::vector<cv::Point3f> &p3ds,
                      const std::vector<cv::Point2f> &p2ds,
                      const CameraPtr &cam,
                      Eigen::Matrix4d &Twc, cv::Mat &img_raw);

    bool calcCamPose(const double &timestamps, const cv::Mat &image,
                     const CameraPtr &cam, Eigen::Matrix4d &Twc);

    CalibrBoardInfo CalBoardInfo_;

};

