#ifndef TRACKER_H
#define TRACKER_H

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>


#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class Tracker
{
public:
    Tracker();
    void getTransformation(const cv::Mat & image, Eigen::Affine3f & Matrix);
//private:
    void getTransformOpenCVChessboard(const cv::Mat & image, Eigen::Affine3f & Matrix);
    void getTransformAruCoChessboard(const cv::Mat & image, Eigen::Affine3f & Matrix);
    Eigen::Affine3f create_rotation_matrix(double ax, double ay, double az);
    string TheIntrinsicFile;
    string TheBoardConfigFile;
    float TheMarkerSize;
    aruco::CameraParameters TheCameraParameters;
    aruco::BoardConfiguration TheBoardConfig;
    aruco::BoardDetector TheBoardDetector;
    double ThresParam1;
    double ThresParam2;
    int iThresParam1;
    int iThresParam2;
    static void cvTackBarEvents(int pos, void *obj);
    Eigen::Affine3f createMatrixfromVectors(const cv::Mat &rvec, const cv::Mat &tvec);
    float conversionmultiplier;
};

#endif // TRACKER_H
