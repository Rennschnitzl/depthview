#ifndef CAMERADRIVER_H
#define CAMERADRIVER_H

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

#define REALSENSE_LASER_CTRL V4L2_CID_PRIVATE_BASE
#define REALSENSE_PATTERN_CTRL (V4L2_CID_PRIVATE_BASE + 1)
#define REALSENSE_EXPOSURE_CTRL (V4L2_CID_PRIVATE_BASE + 2)
#define REALSENSE_FILTER_CTRL (V4L2_CID_PRIVATE_BASE + 3)
#define REALSENSE_CONFIDENCE_CTRL (V4L2_CID_PRIVATE_BASE + 4)


using namespace std;
using namespace cv;

// TODO check for memory leak with qmenu when clearing control struct
struct control {
    struct v4l2_queryctrl qctrl;
    //QList<struct v4l2_querymenu> qmenu;
};

class CameraDriver
{

public:
    explicit CameraDriver(std::string devicepath, u_int32_t fourcc);
    ~CameraDriver();

    int fd;
    std::string device;
    u_int32_t fourcc;
    enum State {
        OPEN =           (1u << 0),
        FMT =            (1u << 1),
        REQBUFS  =       (1u << 2),
        BUFFERS_ARRAY  = (1u << 3),
        QUERYBUF  =      (1u << 4),
        MMAP  =          (1u << 5),
        QBUF  =          (1u << 6),
        STREAM  =        (1u << 7),
        TIMER  =         (1u << 8),
    };
    struct v4l2_format v4l2Format;
    State state;

    bool closeCamera();
    bool openCamera();
    bool startVideo();
    void stopVideo();
    void printState();
    bool setFormat();
    bool reqBuffers();
    bool newBufArray();
    bool queryAllBuffers();
    bool mMAP();
    bool qbuf();
    bool freeMmap();
    bool freeBufferArray();
    bool freeBuffers();
    bool startStream();
    bool stopStream();


    int updateData(Mat *rgb);
    int updateData(cv::Mat * depthmat, cv::Mat * irmat);
    void createImages(void * voidData);







    //QTextStream out;
    //QString device;
    //QString snapshotDir;

    //QImage colorImage;
    //QImage depthImage;
    //QImage infraredImage;
    bool takeSnap;
    int fileFormat;

    __u32 buffercount;
    v4l2_buffer * buffers;
    struct v4l2_requestbuffers reqestBuffers;
    //QTimer *timer;
//    QList<struct v4l2_queryctrl> * depthControlList;
    //QList<struct control> * controlList;

    void openFifo();
    bool getControls();
    bool startClock();
    bool stopClock();


    enum PixelFormat {
        YUYV = 0x56595559,
        INVZ = 0x5a564e49,
        INZI = 0x495a4e49,
        INVR = 0x52564e49,
        INRI = 0x49524e49,
        INVI = 0x49564e49,
        RELI = 0x494c4552
    };

//public:

    __s32 getControl(__u32);
    void setControl(__u32,__s32);
    // herpaderp


    void setCameraDevice(String);
    void setFourcc(u_int32_t);
    void setLaserPower(int);
    void setIvcamSetting(int);
    void setMrtoSetting(int);
    void setFilterSetting(int);
    void setConfidenceSetting(int);
    void savePicture();
    void setFileFormat(int);
    void setSnapshotDir(String);
};

#endif // CAMERADRIVER_H
