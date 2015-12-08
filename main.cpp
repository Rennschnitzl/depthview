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
#include "display.h"

using namespace std;

int main()
{
    CameraWrapper cw;
    Display disp;

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

        Converter::analyseStack(depthlist, result_zeroes, result_depth);
        Converter::undistortDepth(result_depth);

        disp.displayDepth(result_depth);
        disp.displayDepthCM(result_depth);

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
            viewer->updatePointCloud(cloud, "sample cloud");
            viewer->spin();
        }

    }

    cw.shutdownCamera();


    return 0;
}

