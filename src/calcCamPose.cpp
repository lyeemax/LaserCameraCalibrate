#include "calcCamPose.h"
using namespace std;
void CamPoseEst::FindTargetCorner(cv::Mat &img_raw,
                                  vector<vector<cv::Point2f>> &twochesscorners,
                                  vector<vector<cv::Point3f>> &twochesscorners3d)
{

    std::cout << "CHESSBOARD\n";
    const int col = CalBoardInfo_.cols_;
    const int row = CalBoardInfo_.rows_;
    const float square_size = CalBoardInfo_.tagSize_;

    vector<cv::Point> corners_p;//存储找到的角点

    std::vector<cv::Mat> chessboards;
    CornerDetAC corner_detector(img_raw);
    ChessboradStruct chessboardstruct;

    Corners corners_s;
    corner_detector.detectCorners(img_raw, corners_p, corners_s, 0.01);
    ImageChessesStruct ics;
    chessboardstruct.chessboardsFromCorners(corners_s, chessboards, 0.6);
    assert(chessboards.size()==2);
    twochesscorners.resize(2);
    for (int k = 0; k < chessboards.size(); k++)
    {
        for (int i = 0; i < chessboards[k].rows; i++){
            for (int j = 0; j < chessboards[k].cols; j++)
            {
                int d = chessboards[k].at<int>(i, j);
                twochesscorners[k].push_back(cv::Point2f(corners_s.p[d].x, corners_s.p[d].y));
            }
        }

    }

    twochesscorners3d.resize(2);

    for (int l = 0; l <2 ; ++l) {
        cv::cornerSubPix(
                img_raw, twochesscorners[l], cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        if (twochesscorners[l].size() == col * row)
        {
            for (int i = 0; i < row; i++)
            {
                for (int j = 0; j < col; j++)
                {
                    // Todo: change 3d-coordinate
                    twochesscorners3d[l].emplace_back(
                            cv::Point3f(j * square_size, i * square_size, 0.0));
                }
            }
        }
        else
        {
            std::cout << "Chessboard config is not correct with image\n";
        }
    }
}

bool CamPoseEst::EstimatePose(const std::vector<cv::Point3f> &p3ds,
                  const std::vector<cv::Point2f> &p2ds,
                  const CameraPtr &cam,
                  Eigen::Matrix4d &Twc, cv::Mat &img_raw)
{
  Twc.setIdentity();
  if (p3ds.size() != p2ds.size() || p3ds.size() < 4)
  {
    return false;
  }

  cv::Mat_<float> K = (cv::Mat_<float>(3, 3) << 1, 0.0, 0, 0.0, 1, 0, 0.0, 0.0, 1.0);
  cv::Mat_<float> dist = (cv::Mat_<float>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
  cv::Mat cv_r, cv_t;
  cv::Mat inliers;
  cv::solvePnP(p3ds, p2ds, K, dist, cv_r, cv_t);
  cv::Mat rotation;
  cv::Rodrigues(cv_r, rotation);
  Eigen::Matrix3d Rcw;
  cv::cv2eigen(rotation, Rcw);
  Eigen::Vector3d tcw;
  cv::cv2eigen(cv_t, tcw);
  Twc.block<3, 3>(0, 0) = Rcw.inverse();
  Twc.block<3, 1>(0, 3) = -Rcw.inverse() * tcw;

  std::cout << "twc: "<<Twc(0, 3)<<" "<<Twc(1, 3)<<" "<<Twc(2, 3)<<std::endl;
  cv::putText(
      img_raw, "t_wc: (m) " + std::to_string(Twc(0, 3)) + " " + std::to_string(Twc(1, 3)) + " " + std::to_string(Twc(2, 3)),
      cv::Point2f(50, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));

//  std::vector<cv::Point3f> axis;
//  std::vector<cv::Point2f> imgpts;
//  axis.push_back(cv::Point3f(0,0,0));
//  axis.push_back(cv::Point3f(1,0,0));
//  axis.push_back(cv::Point3f(0,1,0));
//  axis.push_back(cv::Point3f(0,0,1));
//  cv::projectPoints(axis, cv_r, cv_t, K, dist,imgpts);

  std::vector<Eigen::Vector3d> axis;
  std::vector<cv::Point2f> imgpts;
  axis.push_back(Eigen::Vector3d(0,0,0));
  axis.push_back(Eigen::Vector3d(0.2,0,0));
  axis.push_back(Eigen::Vector3d(0,0.2,0));
  axis.push_back(Eigen::Vector3d(0,0,0.2));
  for (size_t i = 0; i < axis.size(); ++i) {
    Eigen::Vector2d pt;
    Eigen::Vector3d Pt = Rcw * axis[i] + tcw;
    cam->spaceToPlane(Pt, pt);   // 三维空间点，加上畸变投影到图像平面
    imgpts.push_back(cv::Point2f(pt.x(),pt.y()));
  }

  cv::line(img_raw, imgpts[0], imgpts[1], cv::Scalar(255,0,0), 2);
  cv::line(img_raw, imgpts[0], imgpts[2], cv::Scalar(0,255,0), 2);
  cv::line(img_raw, imgpts[0], imgpts[3], cv::Scalar(0,0,255), 2);

  return true;
}