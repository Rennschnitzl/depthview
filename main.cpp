#include <iostream>
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

#include "cameradriver.h"

using namespace std;

int main()
{
    CameraDriver *colorcam = new CameraDriver("/dev/video1", 0x56595559);
    CameraDriver *depthcam = new CameraDriver("/dev/video2", 0x49524e49);

    //colorcam->openCamera();
    //colorcam->openCamera("/dev/video2", 0x495a4e49);
    //colorcam->printState();

    colorcam->startVideo();
    depthcam->startVideo();

    sleep(5);
    cv::Mat rgbimage, irimage, depthimage;

    namedWindow( "rgb window", WINDOW_AUTOSIZE );
    namedWindow( "depth window", WINDOW_AUTOSIZE );
    namedWindow( "ir window", WINDOW_AUTOSIZE );

    for( int a = 1; a < 20; a = a + 1 )
    {
        colorcam->updateData(&rgbimage);
        depthcam->updateData(&depthimage, &irimage);

        cout << "next image" << endl;

        imshow( "rgb window", rgbimage );
        imshow( "depth window", depthimage );
        imshow( "ir window", irimage );
        waitKey(0);
    }
//    cout << "waiting 2" << endl;
//    sleep(5);
//    cout << "enougth waiting" << endl;
    colorcam->stopVideo();
    //colorcam->closeCamera();
    sleep(5);
    cout << "Hello World!" << endl;


    return 0;
}

