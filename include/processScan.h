//
// Created by unicorn on 2020/9/22.
//

#ifndef LASERCAMERACALIBRATE_PROCESSSCAN_H
#define LASERCAMERACALIBRATE_PROCESSSCAN_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Core>
#include <math.h>
#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
using namespace std;
Eigen::Matrix3d cross(Eigen::Vector3d x){
    Eigen::Matrix3d X;
    X<<0,-x[2],x[1],
            x[2],0,-x[0],
            -x[1],x[0],0;
    return X;
}

class LaserUtil{
public:
    vector<Eigen::Vector2d> Points;
    pair<vector<Eigen::Vector3d>,vector<Eigen::Vector3d>> twoLinesRes;
    pcl::PointCloud<pcl::PointXYZRGB> pclLines;
private:
    bool visualize;
public:
    LaserUtil(bool visualize_):visualize(visualize_){};

    void LaserCallBack(sensor_msgs::LaserScanConstPtr laser){
        sensor_msgs::LaserScan scan;
        twoLinesRes.first.clear();
        twoLinesRes.second.clear();
        bool res=ProcessLaser(*laser,scan);
        if(!res){
            cout<<"No line detect"<<endl;
        }else{
            if(!visualize) return ;
            vector<Eigen::Vector3d> Pts1=twoLinesRes.first,Pts2=twoLinesRes.second;
            sensor_msgs::PointCloud2 plines;

            pclLines.resize(Pts1.size()+Pts2.size());
            int n=0;
            for (int j =0 ; j <Pts1.size()*0.8 ; ++j) {
                pclLines[n].x=Pts1[j].x();
                pclLines[n].y=Pts1[j].y();
                pclLines[n].z=0;
                pclLines[n].b=255;
                n++;
            }
            for (int k = 0; k <Pts2.size()*0.8 ; ++k) {
                pclLines[n].x=Pts2[k].x();
                pclLines[n].y=Pts2[k].y();
                pclLines[n].z=0;
                pclLines[n].g=255;
                n++;
            }

            pcl::toROSMsg(pclLines,plines);
            plines.header.frame_id="pclLines";

            //PclPub.publish(plines);
        }
    }

    bool ProcessLaser(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out){
        scan_out.angle_max=scan_in.angle_max;
        scan_out.angle_min=scan_in.angle_min;
        scan_out.angle_increment=scan_in.angle_increment;
        scan_out.time_increment=scan_in.time_increment;
        scan_out.range_max=scan_in.range_max;
        scan_out.range_min=scan_in.range_min;
        scan_out.intensities=scan_in.intensities;
        scan_out.ranges.resize(scan_in.ranges.size());
        scan_out.header.frame_id="laser_cut";

        double angleMax=45.0*M_PI/180.0;
        double angleMin=-45.0*M_PI/180.0;
        for (int j = 0; j <scan_in.ranges.size() ; ++j) {
            double theta=scan_in.angle_min + (double) j* scan_in.angle_increment;
            if((theta>angleMin && theta<angleMax) ||scan_in.ranges[j]<1.0 ){
                scan_out.ranges[j]=scan_in.ranges[j];
            }else{
                scan_out.ranges[j]=INFINITY;
            }
        }
        ScanToPoints(scan_out,Points);
        return GetLines(Points,twoLinesRes);
    }
    void ScanToPoints(const sensor_msgs::LaserScan& scan_in,std::vector<Eigen::Vector2d>& Points)
    {
        for (int j = 0; j <scan_in.ranges.size() ; ++j) {
            double theta=scan_in.angle_min + (double) j* scan_in.angle_increment;
            Eigen::Vector2d p;
            if(scan_in.ranges[j]<1.0 ){
                p.x()=(double) scan_in.ranges[j]*cos(theta);
                p.y()=(double) scan_in.ranges[j]*sin(theta);
                Points.push_back(p);
            }else{
                p.x()=INFINITY;
                p.y()=INFINITY;
                Points.push_back(p);
            }
        }
    }

    bool GetLines(const std::vector<Eigen::Vector2d> points,pair<vector<Eigen::Vector3d>,vector<Eigen::Vector3d>> &twoLinesRes){

        double maxdist=1.5;
        double mindist=0.5;
        std::vector<Eigen::Vector2d> tempLinePts;
        int count=0;
        for (int i = 0; i <points.size() ; ++i) {
            double d=points[i].norm();
            if(d>=mindist && d<=maxdist){
                tempLinePts.push_back(points[i]);
                count++;
            }else{
                continue;
            }
        }
        if(count<20) return false;

        //先找连续点分类
        vector<vector<Eigen::Vector2d>> ContinuesPoints;
        Eigen::Vector2d lastPt=tempLinePts[0];
        double condist=0.05;
        int index=0;
        ContinuesPoints.push_back(vector<Eigen::Vector2d>());
        ContinuesPoints[0].push_back(lastPt);

        for (int j = 1; j <tempLinePts.size() ; ++j) {
            double t=fabs((tempLinePts[j]-lastPt).norm());
            if(t>condist){
                vector<Eigen::Vector2d> vp;
                vp.push_back(tempLinePts[j]);
                ContinuesPoints.push_back(vp);
                index++;
            }else{
                ContinuesPoints[index].push_back(tempLinePts[j]);
            }
            lastPt=tempLinePts[j];
        }

        //delete continues set which has less than 20 points
        vector<vector<Eigen::Vector2d>>::iterator it,itend,itmax;
        it=ContinuesPoints.begin();
        itend=ContinuesPoints.end();

        int cmax=-1;
        for (;it!=itend;it++) {
            int s=it->size();
            if(s>cmax){
                cmax=s;
                itmax=it;
            }
        }

        vector<Eigen::Vector2d> CandidatePoints=*itmax;
        uniform_int_distribution<int> dis1(cmax*0.1,cmax*0.35);
        uniform_int_distribution<int> dis2(cmax*0.55,cmax*0.90);
        random_device device;
        vector<Eigen::Vector3d> VLP1,VLP2;
        int iterateNum=0;
        while(true){
            iterateNum++;
            if(iterateNum>500){
                cout<<"No Lines Detected!"<<endl;
                break;
            }
            int id1=dis1(device);
            int id2=dis1(device);
            int id3=dis2(device);
            int id4=dis2(device);
            if(id1==id2 || id3==id4) continue;
            Eigen::Vector3d pt1;
            pt1<<CandidatePoints[id1],1;
            Eigen::Vector3d pt2;
            pt2<<CandidatePoints[id2],1;
            Eigen::Vector3d pt3;
            pt3<<CandidatePoints[id3],1;
            Eigen::Vector3d pt4;
            pt4<<CandidatePoints[id4],1;
            Eigen::Vector3d L1=cross(pt1)*pt2;
            Eigen::Vector3d L2=cross(pt3)*pt4;
            VLP1.clear();
            VLP2.clear();
            int repeat=0;
            for (int j = 0; j <cmax ; ++j) {
                Eigen::Vector3d p3dj=Eigen::Vector3d(CandidatePoints[j].x(),CandidatePoints[j].y(),1.0);
                double d1=1000.0*p3dj.transpose()*L1;
                double d2=1000.0*p3dj.transpose()*L2;
                if(fabs(d1)<0.1 && abs(d2)>0.2) VLP1.push_back(p3dj);
                else if(fabs(d1)>0.2 && abs(d2)<0.1) VLP2.push_back(p3dj);
                else if(fabs(d1)<0.05 && abs(d2)<0.05){
                    VLP2.push_back(p3dj);
                    VLP1.push_back(p3dj);
                    repeat++;
                }
                if(double(repeat)/double(cmax)>0.005) break;
            }
            int count=VLP1.size()+VLP2.size();
            if(count>0.7*cmax){
                double rate=double(VLP1.size())/double(count);
                if(rate>0.4 && rate<0.6) {
                    cout<<"found succeed with one of rate:"<< rate*100<<"%"<<endl;
                    break;
                }
                else continue;
            }
        }

        twoLinesRes=make_pair(VLP1,VLP2);
        return true;
    }
};
#endif //LASERCAMERACALIBRATE_PROCESSSCAN_H
