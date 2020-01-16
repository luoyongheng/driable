//
// Created by lyh on 2020/1/7.
//

#ifndef DRIVABLE_MAP_H
#define DRIVABLE_MAP_H
#include <iostream>
#include <vector>

#include <pangolin/pangolin.h>

#include "ImageManage.h"

using namespace std;
class Map{
public:
    vector<ImageManage> vecImage;
    void MapShow(const Eigen::Matrix3d& K){
        pangolin::CreateWindowAndBind("Main",640,480);
        glEnable(GL_DEPTH_TEST);
        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(2000,2000,K(0,0),K(1,1),K(0,2),K(1,2),0.2,100),
                //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
                pangolin::ModelViewLookAt(-13,8,-10,0,0,0,0,-1,0)
        );
        pangolin::Handler3D handler(s_cam);
        //setBounds 跟opengl的viewport 有关
        //看SimpleDisplay中边界的设置就知道
        pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0,1.0,0.0,1.0,-640.0f/480.0f)
                .SetHandler(&handler);

        while(!pangolin::ShouldQuit())
        {
            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);

            // Render OpenGL Cube
//        pangolin::glDrawColouredCube();\
        //坐标轴的创建
            pangolin::glDrawAxis(3);

            //点的创建
            glPointSize(10.0f);
            glBegin(GL_POINTS);
            glColor3f(1.0,1.0,1.0);
            for(auto image : vecImage){
                for(auto pts : image.GetPointsSpace()){
                    glVertex3f(pts.x,pts.y,pts.z);
                }
            }
            glEnd();

            //把下面的点都做一次旋转变换
            //glPushMatrix();
            //col major
            //std::vector<GLfloat > Twc = {1,0,0,0, 0,1,0,0 , 0,0,1,0 ,0,0,5,1};
            //glMultMatrixf(Twc.data());

            //直线的创建
            const float w = 0.5;
            const float h = w*0.75;
            const float z = w*0.6;
            glLineWidth(2);
            glColor3f(0,0,1);
            glBegin(GL_LINES);

            //camera
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);
            glEnd();

            glBegin(GL_LINE_STRIP);
            glVertex3f(w,h,z);
            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);;
            glVertex3f(w,-h,z);
            glVertex3f(w,h,z);
            glEnd();

            glBegin(GL_LINE_STRIP);
            //space
            glColor3f(1,0,0);
            for(auto image : vecImage){
                vector<cv::Point3d> ptsSpace(image.GetPointsSpace());
                for(auto it = ptsSpace.begin();it < ptsSpace.end();it++){
                    glVertex3f((*it).x,(*it).y,(*it).z);
                }
            }
            glEnd();

            //TODO OpenGL对凹多边形显示有问题,需要改善
            glColor3f(0,1,0);
            glBegin(GL_POLYGON);
            for(auto image : vecImage){
                vector<cv::Point3d> ptsSpace(image.GetPointsSpace());
                for(auto it = ptsSpace.begin();it < ptsSpace.end();it++){
                    glVertex3f((*it).x,(*it).y,(*it).z);
                }
            }
            glEnd();
            //glPopMatrix();

            // Swap frames and Process Events
            pangolin::FinishFrame();
        }
    }

private:
};
#endif //DRIVABLE_MAP_H
