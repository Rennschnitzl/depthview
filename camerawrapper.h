#ifndef CAMERAWRAPPER_H
#define CAMERAWRAPPER_H

#include <opencv2/opencv.hpp>
#include "cameradriver.h"

class CameraWrapper
{
public:
    CameraWrapper();
    void loadCameraMatrix();
    void displayCameraProperties();
    void initCamera();
    void clearBuffer();
    void shutdownCamera();
    std::vector<cv::Mat> recordStack(int frames);
    struct timespec slptm;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    CameraDriver *depthcam;
};

#endif // CAMERAWRAPPER_H
