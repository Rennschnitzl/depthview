#include "camerawrapper.h"


CameraWrapper::CameraWrapper()
{
    std::cout << "Loading camera matrix" << std::endl;
    loadCameraMatrix();
    slptm.tv_sec = 0;
    slptm.tv_nsec = 50000000;      //1000 ns = 1 us
    initCamera();
}

void CameraWrapper::loadCameraMatrix()
{
    std::cout << "OpenCV version : " << CV_VERSION << std::endl;

    // read settings from file
    cv::FileStorage fs2("calibration.yml", cv::FileStorage::READ);
    fs2["cameraMatrix"] >> cameraMatrix;
    fs2["distCoeffs"] >> distCoeffs;

    // Get some infos from the calibration matrix
    displayCameraProperties();
}

void CameraWrapper::displayCameraProperties()
{
    double apertureWidth;
    double apertureHeight;
    double fovx;
    double fovy;
    double focalLength;
    cv::Point2d principalPoint;
    double aspectRatio;
    calibrationMatrixValues(cameraMatrix, cvSize(640,480), apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectRatio);
    std::cout << "Camera-Matrix Data:" << "\nAperture (W/H): " << apertureWidth << " / " << apertureHeight << "\nFoV (x/y): "
              << fovx << " / " << fovy << "\nPrincipal point: " << principalPoint << "\nAspect ratio: " << aspectRatio << std::endl;
}

void CameraWrapper::initCamera()
{
    //CameraDriver *colorcam = new CameraDriver("/dev/video1", 0x56595559);
    depthcam = new CameraDriver("/dev/video2", 0x49524e49);
    //colorcam->startVideo();
    if(!depthcam->startVideo())
        std::cout << "ALLES KACKE, NIX GEHT" << std::endl;
    sleep(1);
}

void CameraWrapper::clearBuffer()
{
    cv::Mat rgbimage, irimage, depthimage;
    // clear buffer
    for(int i = 0; i<3; i++)
    {
        nanosleep(&slptm,NULL);
        //colorcam->updateData(&rgbimage);
        depthcam->updateData(&depthimage, &irimage);
    }
}

void CameraWrapper::shutdownCamera()
{
    //colorcam->stopVideo();
    depthcam->stopVideo();
}

std::vector<Mat> CameraWrapper::recordStack(int frames)
{
    clearBuffer();

    cv::Mat rgbimage, irimage, depthimage;
    std::vector<cv::Mat> irlist, depthlist;
    // grab image vectors
    //irlist.clear();
    depthlist.clear();
    for(int i = 0; i<frames; i++)
    {
        nanosleep(&slptm,NULL);
        //colorcam->updateData(&rgbimage);
        depthcam->updateData(&depthimage, &irimage);
        //irlist.push_back(irimage.clone());
        depthlist.push_back(depthimage.clone());
    }

    return depthlist;
}


