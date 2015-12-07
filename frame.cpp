#include "frame.h"

Frame::Frame(std::vector<cv::Mat> rawStackDepth, cv::Mat cameraMatrix, Eigen::Vector4f origin, Eigen::Quaternionf orientation)
{
    // zuweisungen
    // set converter
    // set transform
    // convert
}

void Frame::convert()
{
    // convert stack to belief / 2d
    // convert 2d to pcl
}

cv::Mat Frame::getImage()
{
    // return image
}

pcl::PointXYZ Frame::getCloud()
{
    // return cloud
}

//void Frame::setConverter()
//{

//}
