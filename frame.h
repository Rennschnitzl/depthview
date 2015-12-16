#ifndef FRAME_H
#define FRAME_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>
//#include <pcl/point_types.h>
#include "converter.h"
#include <pcl/common/transforms.h>

class Frame
{
public:
    // TODO: add converter function pointer
    Frame(std::vector<cv::Mat> rawStackDepth, std::vector<cv::Mat> rawStackIR, cv::Mat cameraMatrix, Eigen::Affine3f &matrix);
    //void setConverter();
    Eigen::Affine3f gettransform();
    void settransform(Eigen::Affine3f matrix);
    void convert();
    cv::Mat getImage();
    cv::Mat getIRImage();
    pcl::PointCloud<pcl::PointXYZ> getCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudPointer();
//    pcl::PointCloud<pcl::PointXYZ> getTransformedCloud();
private:
    //Converter
    cv::Mat cameraMatrix;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    Eigen::Affine3f matrix;
    cv::Mat belief;
    cv::Mat processedImageDepth;
    cv::Mat processedImageIR;
    std::vector<cv::Mat> rawStackIR;
    std::vector<cv::Mat> rawStackDepth;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr;    
};

#endif // FRAME_H
