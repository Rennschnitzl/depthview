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
#include "camerawrapper.h"
#include "display.h"
#include "frame.h"
#include <sstream> // std::to_string
#include <string>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include "tracker.h"


using namespace std;

template <typename T>
std::string to_string(T value)
{
  //create an output string stream
  std::ostringstream os ;

  //throw the value into the string stream
  os << value ;

  //convert the string stream into a string and return
  return os.str() ;
}

int main()
{
    CameraWrapper cw;
    Display disp;
    Tracker track;

    /// trackingsurrogat :D
//    Eigen::Vector4f origin;
//    Eigen::Quaternionf orientation;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "preview");
    //viewer->addCoordinateSystem (2.0);
    viewer->initCameraParameters ();
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "preview");
    viewer->addCoordinateSystem(1.0, "marker");

    int key;
    int savedframes = 0;
    std::vector<Frame> savedFrameObjects;



    while(true)
    {

        std::vector<Mat> irlist, depthlist;
        cw.recordStack(5,irlist, depthlist);
        Frame fr = Frame(depthlist, irlist, cw.cameraMatrix, transform);

        fr.convert();

        disp.displayDepth(fr.getImage());
        disp.displayDepthCM(fr.getImage());
        disp.displayIR(fr.getIRImage());

        cv::imwrite("depthimage.png", fr.getImage());
        cv::imwrite("irimage.png", fr.getIRImage());

        Eigen::Affine3f mpose;
        track.getTransformation(fr.getIRImage(), mpose);
        fr.settransform(mpose);



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
            //viewer->addPointCloud(fr.getCloudPointer(), "derpy cloud");
            viewer->updateCoordinateSystemPose("marker", mpose.inverse());
            viewer->updatePointCloud(fr.getCloudPointer(), "preview");
            viewer->updatePointCloudPose("preview", mpose.inverse());
            viewer->spin();
        }
        // "s"
        if(key == 115)
        {
            savedframes++;
            savedFrameObjects.push_back(fr);
            viewer->addPointCloud(fr.getCloudPointer(), to_string(savedframes));
            viewer->updatePointCloudPose(to_string(savedframes), fr.gettransform().inverse());
            viewer->addCoordinateSystem(3.0, fr.gettransform().inverse(), to_string(savedframes));
        }

    }

    cw.shutdownCamera();


    return 0;
}

