#include "display.h"

Display::Display()
{
    namedWindow( "depth", cv::WINDOW_AUTOSIZE );
    //namedWindow( "ir", cv::WINDOW_AUTOSIZE );
    namedWindow( "depth colormap", cv::WINDOW_AUTOSIZE);
}

void Display::displayDepth(const cv::Mat & image)
{
//    std::cout << "Distance in center: " << (int)depthimage.at<ushort>(240,320) << std::endl;
//    depthimage.at<ushort>(240,320) = 65000;
    cv::imshow( "depth", image);
}

void Display::displayDepthCM(const cv::Mat & image)
{
    cv::Mat depth_cv;
    image.convertTo(depth_cv,CV_8U,1.0/256.0);

    cv::Mat cm_depth;
    cv::applyColorMap(depth_cv, cm_depth, cv::COLORMAP_HSV);
    cv::imshow( "depth colormap", cm_depth);
}

void Display::displayIR(const cv::Mat & image)
{
    cv::imshow( "ir", image);
}

cv::Mat Display::rectify(const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs, const cv::Mat & image)
{
    cv::Mat view, rview, map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                            cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image.size(), 1, image.size(), 0),
            image.size(), CV_16SC2, map1, map2);
    view = image;
    cv::remap(view, rview, map1, map2, cv::INTER_LINEAR);
    return rview;
}

