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
    namedWindow( "depth window", WINDOW_AUTOSIZE );
    namedWindow( "ir window", WINDOW_AUTOSIZE );
    namedWindow( "ir averages", WINDOW_AUTOSIZE);
    int key;

    std::vector<cv::Mat> irlist, depthlist;
    cv::Mat result(480, 640, CV_8U, Scalar::all(0));

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
        for(int i = 0; i<20; i++)
        {
            nanosleep(&slptm,NULL);
            //colorcam->updateData(&rgbimage);
            depthcam->updateData(&depthimage, &irimage);
            irlist.push_back(irimage.clone());
            depthlist.push_back(depthimage.clone());
        }

        std::cout << irlist[0].rows << " - " << irlist[0].cols << std::endl;

        for (int i = 0; i < irlist[0].rows; i++)
        {
            for (int j = 0; j < irlist[0].cols; j++)
            {
                int tempresult = 0;
                for(int il = 0; il < irlist.size(); il++) {
                    //std::cout << i << " " << j << " " << il << std::endl;
                    tempresult += (int)irlist[il].at<uchar>(i,j);
                }
                result.at<uchar>(i,j) = tempresult/irlist.size();
            }
        }


//        cv::imwrite("depth.png", depthimage);
        std::cout << "Distance in center: " << (int)depthimage.at<ushort>(240,320) << std::endl;
        depthimage.at<ushort>(240,320) = 65000;

        //imshow( "rgb window", rgbimage );
        imshow( "depth window", depthimage );
        imshow( "ir window", irlist[0] );
        imshow( "ir averages", result);



        // a => stop; rest => next frame
        key = waitKey(0);
        if(key == 1048673)
            break;
    }

    //colorcam->stopVideo();
    depthcam->stopVideo();


    return 0;
}

