#include <iostream>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>
#include <boost/thread/thread.hpp>
#include "converter.h"
#include "camerawrapper.h"

using namespace std;

// TODO: call by reference und so
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


int main()
{
    CameraWrapper cw;

    cw.initCamera();



    //namedWindow( "rgb window", WINDOW_AUTOSIZE );
    namedWindow( "depth", WINDOW_AUTOSIZE );
    namedWindow( "ir", WINDOW_AUTOSIZE );
    namedWindow( "ir averages", WINDOW_AUTOSIZE);
    namedWindow( "depth averages", WINDOW_AUTOSIZE);
    namedWindow( "zeroes", WINDOW_AUTOSIZE);


    std::vector<cv::Mat> depthlist;
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

    int key;

    while(true)
    {
        // muss ich das clearen? überschreibs ja. keine ahnung :(
        depthlist.clear();
        depthlist = cw.recordStack(40);

        std::cout << "Distance in center: " << (int)depthlist[0].at<ushort>(240,320) << std::endl;

        Converter::analyseStack(depthlist, result_zeroes, result_depth);
        Converter::undistortDepth(result_depth);


        //displayImages(result_zeroes, result_ir, depthimage, distCoeffs, irlist, result_depth, cameraMatrix);


        cloud->clear();
        Converter::depthTo3d(result_depth,cw.cameraMatrix,cloud);

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
            //viewer->updatePointCloud(cloud, "sample cloud");
            //viewer->spin();
//            while (!viewer->wasStopped ())
//            {
//                viewer->spinOnce (100);
//                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//            }
        }

    }

    cw.shutdownCamera();


    return 0;
}

