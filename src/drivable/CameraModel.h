//
// Created by lyh on 2020/1/6.
//

#ifndef DRIVABLE_CAMERAMODEL_H
#define DRIVABLE_CAMERAMODEL_H

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
class Camera{
public:
    static Camera& GetInstance(const string& fileConfig){
        static Camera camera(fileConfig);
        return camera;
    }
    Eigen::Matrix3d GetIntrisic(){
        return K;
    }
    double GetCameraHeight(){
        return camH;
    }
private:
    Camera(const string& fileConfig){
        SetParameters(fileConfig);
        K << mfx,0,mu0,
            0,mfy,mv0,
            0,0,1;
    }
    void SetParameters(const string& fileConfig) {
        cv::FileStorage fConfig(fileConfig, cv::FileStorage::READ);
        if(!fConfig.isOpened()) {
            cerr << "cannot find file: " << fileConfig << endl;
            return;
        }
        cv::FileNode imageSize = fConfig["image_size"];
        mWidth = (int)(imageSize["width"]);
        mHeight = (int)(imageSize["height"]);

        cv::FileNode intrinsicList = fConfig["intrinsic_list"];
        mfx = (double)(intrinsicList["fpx"]);
        mfy = (double)(intrinsicList["fpy"]);
        mu0 = (double)(intrinsicList["cx"]);
        mv0 = (double)(intrinsicList["cy"]);

        cv::FileNode distortionList = fConfig["distortion_list"];
        int i = 0;
        for(auto it : distortionList){
            distort[i] = (double)it;
            i++;
            if(i > 8)
                break;
        }

        double num[16];
        cv::FileNode extrinsicList = fConfig["extrinsic_list"];
        int j = 0;
        for(auto it : extrinsicList["extrinsic_matrix"]){
            num[j] = (double)it;
            j++;
        }
        Tcw << num[0],num[1],num[2],num[3],
               num[4],num[5],num[6],num[7],
               num[8],num[9],num[10],num[11],
               num[12],num[13],num[14],num[15];
    }

    double mfx,mfy,mu0,mv0;
    typedef Eigen::Matrix<double,8,1> Vector6d;
    //Vector6d distort;
    Vector6d distort;
    Eigen::Matrix4d Tcw;
    int mWidth, mHeight;
    Eigen::Matrix3d K;
    double camH = 1.313;

};
#endif //DRIVABLE_CAMERAMODEL_H
