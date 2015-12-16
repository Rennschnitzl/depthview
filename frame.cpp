#include "frame.h"

/*
cv::Mat belief;
cv::Mat processedImageDepth;
std::vector<cv::Mat> rawStackIR;
pcl::PointXYZ cloud;
*/

Frame::Frame(std::vector<cv::Mat> rawStackDepth, std::vector<cv::Mat> rawStackIR, cv::Mat cameraMatrix, Eigen::Affine3f & matrix)
{
    //TODO: converter
    // still dont know if i have to do this :(
    this->rawStackDepth.clear();

    this->rawStackDepth = rawStackDepth;
    this->rawStackIR = rawStackIR;
    this->cameraMatrix = cameraMatrix;
    this->matrix = matrix;
    this->cloudptr = this->cloud.makeShared();

    cv::Mat processedImageIR(480, 640, CV_8U, cv::Scalar::all(0));
    cv::Mat belief(480, 640, CV_8U, cv::Scalar::all(0));
    cv::Mat processedImageDepth(480, 640, CV_16U, cv::Scalar::all(0));

    // maybe clone()
    this->belief = belief;
    this->processedImageDepth = processedImageDepth;
    this->processedImageIR = processedImageIR;
}

Eigen::Affine3f Frame::gettransform()
{
    return this->matrix;
}

void Frame::settransform(Eigen::Affine3f matrix)
{
    this->matrix = matrix;
}

void Frame::convert()
{
    Converter::analyseStack(this->rawStackDepth, this->belief, this->processedImageDepth);
    Converter::averageIR(this->rawStackIR, this->processedImageIR);
    Converter::undistortDepth(this->processedImageDepth);
    Converter::depthTo3d(this->processedImageDepth,this->cameraMatrix,this->cloudptr);
}


cv::Mat Frame::getImage()
{
    return this->processedImageDepth.clone();
}

cv::Mat Frame::getIRImage()
{
    return this->processedImageIR.clone();
}


pcl::PointCloud<pcl::PointXYZ> Frame::getCloud()
{
    return this->cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Frame::getCloudPointer()
{
    return this->cloudptr;
}

//pcl::PointCloud<pcl::PointXYZ> Frame::getTransformedCloud()
//{
//    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
//    pcl::transformPointCloud(this->cloud, transformed_cloud, this->matrix);
//    return transformed_cloud;
//}

//void Frame::setConverter()
//{

//}
