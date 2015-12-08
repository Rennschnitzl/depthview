#ifndef DISPLAY_H
#define DISPLAY_H

#include <opencv2/opencv.hpp>

class Display
{
public:
    Display();
    void displayDepth(const cv::Mat &image);
    void displayIR(const cv::Mat &image);
    cv::Mat rectify(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, const cv::Mat &image);
    void displayDepthCM(const cv::Mat &image);
};

#endif // DISPLAY_H
