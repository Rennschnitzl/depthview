#ifndef CONVERTER_H
#define CONVERTER_H

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
#include <iostream>
#include <string.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

class Converter
{
public:
    Converter();
    static std::string type2str(int type);
    static void analyseStack(std::vector<cv::Mat> stack, cv::Mat believe, cv::Mat result);
    static void undistortDepth(cv::Mat depth);
    static void rescaleDepth(cv::InputArray in_in, int depth, cv::OutputArray out_out);
    static void depthTo3d(const cv::Mat& in_depth, const cv::Mat& K, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif // CONVERTER_H


