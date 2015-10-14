#include <iostream>
#include <opencv2/opencv.hpp>
#include "cameradriver.h"
#include <time.h>
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

int main()
{
    // nanosleep
    struct timespec slptm;
    slptm.tv_sec = 0;
    slptm.tv_nsec = 50000000;      //1000 ns = 1 us


    //CameraDriver *colorcam = new CameraDriver("/dev/video1", 0x56595559);
    CameraDriver *depthcam = new CameraDriver("/dev/video2", 0x49524e49);

    //colorcam->startVideo();
    depthcam->startVideo();

    sleep(1);
    cv::Mat rgbimage, irimage, depthimage;

    //namedWindow( "rgb window", WINDOW_AUTOSIZE );
    namedWindow( "depth", WINDOW_AUTOSIZE );
    namedWindow( "ir", WINDOW_AUTOSIZE );
    namedWindow( "ir averages", WINDOW_AUTOSIZE);
    namedWindow( "depth averages", WINDOW_AUTOSIZE);
    namedWindow( "zeroes", WINDOW_AUTOSIZE);
    int key;

    std::vector<cv::Mat> irlist, depthlist;
    cv::Mat result_ir(480, 640, CV_8U, Scalar::all(0));
    cv::Mat result_zeroes(480, 640, CV_8U, Scalar::all(0));
    cv::Mat result_depth(480, 640, CV_16U, Scalar::all(0));

    while(true)
    {
        // clear buffer
        for(int i = 0; i<3; i++)
        {
            nanosleep(&slptm,NULL);
            //colorcam->updateData(&rgbimage);
            depthcam->updateData(&depthimage, &irimage);
        }

        irlist.clear();
        depthlist.clear();
        // grab image vector
        for(int i = 0; i<40; i++)
        {
            nanosleep(&slptm,NULL);
            //colorcam->updateData(&rgbimage);
            depthcam->updateData(&depthimage, &irimage);
            irlist.push_back(irimage.clone());
            depthlist.push_back(depthimage.clone());
        }


        for (int i = 0; i < irlist[0].rows; i++)
        {
            for (int j = 0; j < irlist[0].cols; j++)
            {
                int tempresult_ir = 0;
                int tempresult_depth = 0;
                int depth_zeroes = 0;
                for(int il = 0; il < irlist.size(); il++) {
                    tempresult_ir += (int)irlist[il].at<uchar>(i,j);
                    tempresult_depth += (int)depthlist[il].at<ushort>(i,j);
                    if((int)depthlist[il].at<ushort>(i,j) == 0)
                        depth_zeroes++;
                }
                result_ir.at<uchar>(i,j) = tempresult_ir/irlist.size();
                if(depthlist.size()-depth_zeroes != 0)
                    result_depth.at<ushort>(i,j) = tempresult_depth/(depthlist.size()-depth_zeroes);
                else
                    result_depth.at<ushort>(i,j) = 0;
                result_zeroes.at<uchar>(i,j) = (int)((240/irlist.size())*depth_zeroes);
            }
        }


//        cv::imwrite("depth.png", depthimage);
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





        // a => stop; rest => next frame
        key = waitKey(0);
        if(key == 1048673)
            break;
    }

    //colorcam->stopVideo();
    depthcam->stopVideo();


    return 0;
}

