#ifndef FRAME_H
#define FRAME_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>
//#include <pcl/point_types.h>
#include "converter.h"

class Frame
{
public:
    Frame(std::vector<cv::Mat> rawStackDepth, cv::Mat cameraMatrix, Eigen::Vector4f origin, Eigen::Quaternionf orientation); // add converter function pointer
    //void setConverter();
    //gettransform()
    //settransform
    void convert();
    cv::Mat getImage();
    pcl::PointXYZ getCloud();
private:
    //Converter
    cv::Mat cameraMatrix;
    //cv::Mat?? Transform
    cv::Mat belief;
    cv::Mat processedImageDepth;
    std::vector<cv::Mat> rawStackIR;
    std::vector<cv::Mat> rawStackDepth;
    pcl::PointXYZ cloud;
};

#endif // FRAME_H
