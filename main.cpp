#include <iostream>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include "cameradriver.h"
#include <time.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>
#include <boost/thread/thread.hpp>
#include "converter.h"


//#include <array>


//#include <string.h>
//#include <linux/videodev2.h>
//#include <linux/uvcvideo.h>
//#include <linux/usb/video.h>
//#include <fcntl.h>
//#include <stdio.h>
//#include <sys/ioctl.h>
//#include <sys/mman.h>
//#include <errno.h>
//#include <unistd.h>


using namespace std;


void displayImages(cv::Mat result_zeroes, cv::Mat result_ir, cv::Mat depthimage, cv::Mat distCoeffs, std::vector<cv::Mat> irlist, cv::Mat result_depth, cv::Mat cameraMatrix)
{


    std::cout << "Distance in center: " << (int)depthimage.at<ushort>(240,320) << std::endl;
    depthimage.at<ushort>(240,320) = 65000;

    // colormap
    // convert for colormapping
    Mat depth_cv_8_avg, depth_cv_8;
    result_depth.convertTo(depth_cv_8_avg,CV_8U,1.0/256.0);
    depthimage.convertTo(depth_cv_8,CV_8U,1.0/256.0);

    Mat cm_depth_avg, cm_depth;
    applyColorMap(depth_cv_8_avg, cm_depth_avg, COLORMAP_HSV  );
    applyColorMap(depth_cv_8, cm_depth, COLORMAP_HSV  );

    //imshow( "rgb window", rgbimage );

    // show depth related
    imshow( "depth", cm_depth );
    imshow( "depth averages", cm_depth_avg);
    //        imshow( "depth", depthimage );
    //        imshow( "depth averages", result_depth);
    imshow( "zeroes", result_zeroes);

    // show ir related
    imshow( "ir", irlist[0] );
    imshow( "ir averages", result_ir);

    // REKTification
    Mat view, rview, map1, map2;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, irlist[0].size(), 1, irlist[0].size(), 0),
            irlist[0].size(), CV_16SC2, map1, map2);
    view = irlist[0];
    remap(view, rview, map1, map2, INTER_LINEAR);
    imshow("Image View", rview);

    imshow("pure depth", depthimage);
}



void displayCameraProperties(cv::Mat cameraMatrix)
{
    double apertureWidth;
    double apertureHeight;
    double fovx;
    double fovy;
    double focalLength;
    Point2d principalPoint;
    double aspectRatio;
    calibrationMatrixValues(cameraMatrix, cvSize(640,480), apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectRatio);
    std::cout << "Camera-Matrix Data:" << "\nAperture (W/H): " << apertureWidth << " / " << apertureHeight << "\nFoV (x/y): "
              << fovx << " / " << fovy << "\nPrincipal point: " << principalPoint << "\nAspect ratio: " << aspectRatio << std::endl;
}

int main()
{

    std::cout << "OpenCV version : " << CV_VERSION << std::endl;

    // read settings from file
    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs2("calibration.yml", cv::FileStorage::READ);
    fs2["cameraMatrix"] >> cameraMatrix;
    fs2["distCoeffs"] >> distCoeffs;

    // Get some infos from the calibration matrix
    displayCameraProperties(cameraMatrix);

    // nanosleep
    struct timespec slptm;
    slptm.tv_sec = 0;
    slptm.tv_nsec = 50000000;      //1000 ns = 1 us

    //CameraDriver *colorcam = new CameraDriver("/dev/video1", 0x56595559);
    CameraDriver *depthcam = new CameraDriver("/dev/video2", 0x49524e49);
    //colorcam->startVideo();
    if(!depthcam->startVideo())
        std::cout << "ALLES KACKE, NIX GEHT" << std::endl;
    sleep(1);

    //namedWindow( "rgb window", WINDOW_AUTOSIZE );
    namedWindow( "depth", WINDOW_AUTOSIZE );
    namedWindow( "ir", WINDOW_AUTOSIZE );
    namedWindow( "ir averages", WINDOW_AUTOSIZE);
    namedWindow( "depth averages", WINDOW_AUTOSIZE);
    namedWindow( "zeroes", WINDOW_AUTOSIZE);

    int key;
    cv::Mat rgbimage, irimage, depthimage;

    std::vector<cv::Mat> irlist, depthlist;
    cv::Mat result_ir(480, 640, CV_8U, Scalar::all(0));
    cv::Mat result_zeroes(480, 640, CV_8U, Scalar::all(0));
    cv::Mat result_depth(480, 640, CV_16U, Scalar::all(0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

    while(true)
    {
        // clear buffer
        for(int i = 0; i<3; i++)
        {
            nanosleep(&slptm,NULL);
            //colorcam->updateData(&rgbimage);
            depthcam->updateData(&depthimage, &irimage);
        }

        // grab image vectors
        irlist.clear();
        depthlist.clear();
        for(int i = 0; i<40; i++)
        {
            nanosleep(&slptm,NULL);
            //colorcam->updateData(&rgbimage);
            depthcam->updateData(&depthimage, &irimage);
            irlist.push_back(irimage.clone());
            depthlist.push_back(depthimage.clone());
        }


        //static void analyseStack(std::vector<cv::Mat> stack, cv::Mat believe, cv::Mat result);

        Converter::analyseStack(depthlist, result_zeroes, result_depth);
        Converter::undistortDepth(result_depth);


        //displayImages(result_zeroes, result_ir, depthimage, distCoeffs, irlist, result_depth, cameraMatrix);


        cloud->clear();
        Converter::depthTo3d(result_depth,cameraMatrix,cloud);

        // a => stop; rest => next frame
        key = waitKey(0);
        // strange opencv workaround
        // see: http://stackoverflow.com/questions/9172170/python-opencv-cv-waitkey-spits-back-weird-output-on-ubuntu-modulo-256-maps-corre
        key -= 0x100000;
        std::cout << key << std::endl;
        if(key == 97)
            break;

        if(key == 119)
        {
            viewer->updatePointCloud(cloud, "sample cloud");
            viewer->spin();
//            while (!viewer->wasStopped ())
//            {
//                viewer->spinOnce (100);
//                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//            }
        }

    }

    //colorcam->stopVideo();
    depthcam->stopVideo();


    return 0;
}

