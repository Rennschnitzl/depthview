#include "cameradriver.h"

CameraDriver::CameraDriver(std::string devicepath, u_int32_t fourcc)
{
    state = (State)0;
    fd = -1;
    //takeSnap = false;
    //controlList = new QList<struct control>();
    memset(&v4l2Format,0,sizeof(v4l2_format));
    //fileFormat = 0;
    //timer = new QTimer(this);
    //connect(timer, SIGNAL(timeout()), this, SLOT(updateData()));
    this->device = devicepath;
    this->fourcc = fourcc;
    printState();

}

CameraDriver::~CameraDriver(){
    //delete timer;
    //delete controlList;
}

void CameraDriver::printState(){
    // I should use array lookup here I know.
    // I kept doing it wrong, and getting compile errors. Lost patience.
    cout << "state: " << (int)state << " states: ";
    for(int i = 0 ; i < 8 ; i++){
        State testState = (State)(1u << i);
        if(state & testState){
            switch(testState){
            case OPEN:
                cout << "OPEN, ";
                break;
            case FMT:
                cout << "FMT, ";
                break;
            case REQBUFS:
                cout << "REQBUFS, ";
                break;
            case BUFFERS_ARRAY:
                cout << "BUFFERS_ARRAY, ";
                break;
            case QUERYBUF:
                cout << "QUERYBUF, ";
                break;
            case MMAP:
                cout << "MMAP, ";
                break;
            case QBUF:
                cout << "QBUF, ";
                break;
            case STREAM:
                cout << "STREAM, ";
                break;
            case TIMER:
                cout << "TIMER, ";
                break;
            default:
                cout << "unknown state";
                break;
            }
        }
    }
    cout << endl;
}


bool CameraDriver::openCamera(){
    cout << "opening Camera" << endl;
    if(state & OPEN)
    {
        cout << "camera already open" << endl;
        return false;
    }
    fd = open(device.c_str(), O_NONBLOCK | O_RDWR); // Open non blocking becauce
    if (fd == -1)                                                 // getting frame from UI thread
    {
        perror("opening device");
        return false;
    }

    state = (State)(state | OPEN);
    return true;
}

bool CameraDriver::closeCamera(){
    cout << "closing camera" << endl;
    uint dMask = OPEN;
    uint cMask = MMAP;
    if(! ((state & dMask) == dMask)){
        cout << "file not open" << endl;
        return false;
    } else if(state & cMask){
        cout << "cannot close program while MMAP" << endl;
        return false;
    }
    if (-1 == close(fd)){
        perror("close camera");
        return false;
    }
    fd = -1;
    state = (State)(state & ~(OPEN | FMT));
    return true;
}

void CameraDriver::startVideo(){
    openCamera();
    //getControls();
    setFormat();
    reqBuffers();
    newBufArray();
    queryAllBuffers();
    mMAP();
    qbuf();
    startStream();
    //startClock();
    printState();
}

void CameraDriver::stopVideo(){
    //stopClock();
    stopStream();
    freeMmap();
    freeBufferArray();
    freeBuffers();
    closeCamera();
    printState();
}

bool CameraDriver::setFormat(){
    cout << "FMT" << endl;
    if(! state & OPEN){
        cout << "must open camera first" << endl;
        return false;
    }else if(state & REQBUFS){
        cout << "cannot set format while buffers active" << endl;
        return false;
    }
    struct v4l2_fmtdesc fmtdesc;
    int i,e,r, trys;
    i = e = r = 0;
    __u8 * pixfmt = new __u8[5];
    pixfmt[4] = 0;
    trys = 255;
    while(trys-- > 0){
        cout << "try fmt" << endl;
        memset(&fmtdesc, 0, sizeof(fmtdesc));
        fmtdesc.index = i;
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)){  // Get video format
            perror("enum fmt");
            break;
        }
        *((__u32 *)pixfmt) = fmtdesc.pixelformat;
        printf("index: %d format: %s description: %s \n",
               i, pixfmt /*fmtdesc.pixelformat */, fmtdesc.description);
        i++;
    }
    delete[] pixfmt;
    struct v4l2_format pixFormat;
    memset(&pixFormat, 0, sizeof(pixFormat));           // Clear struct
    pixFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;    // Required
    if (-1 == ioctl(fd, VIDIOC_G_FMT, &pixFormat)){  // Get video format
        perror("getting format");
        return false;
    }
    pixFormat.fmt.pix.pixelformat = fourcc;

    if(fourcc == YUYV){
        pixFormat.fmt.pix.width       = 1920;
        pixFormat.fmt.pix.height      = 1080;
    } else {
        pixFormat.fmt.pix.width       = 640;
        pixFormat.fmt.pix.height      = 480;
    }
    if (-1 == ioctl(fd, VIDIOC_S_FMT, &pixFormat)){  // Set video format
        perror("setting format");
        return false;
    }
    v4l2Format = pixFormat;
    state = (State)(state | FMT);
    return true;
}
bool CameraDriver::reqBuffers(){
    cout << "REQBUFS" << endl;
    uint dMask = OPEN | FMT;
    if(! ((state & dMask) == dMask)){
        cout << "depends OPEN & FMT" << endl;
        return false;
    } else if(state & REQBUFS){
        cout << "REQBUF already done" << endl;
        return false;
    }
    buffercount = 2; // Double buffer

    memset(&reqestBuffers, 0, sizeof(reqestBuffers));
    reqestBuffers.count = buffercount;
    reqestBuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqestBuffers.memory = V4L2_MEMORY_MMAP;

    // this allocates memory, but does not return a pointer to it
    if (-1 == ioctl (fd, VIDIOC_REQBUFS, &reqestBuffers)) {
        perror("getting buffers");
        return false;
    }
    state = (State)(state | REQBUFS);
    return true;
}
bool CameraDriver::freeBuffers(){
    cout << "freebuffers" << endl;
    uint dMask = OPEN | REQBUFS;
    if((state & dMask) != dMask){
        cout << "depends OPEN & REQBUFS" << endl;
        return false;
    } else if(state & MMAP){
        cout << "cannot free buffers while mapped" << endl;
        return false;
    }
    memset(&reqestBuffers, 0, sizeof(reqestBuffers));
    reqestBuffers.count = 0; // Requesting zero buffers frees them
    reqestBuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqestBuffers.memory = V4L2_MEMORY_MMAP;

    if (-1 == ioctl (fd, VIDIOC_REQBUFS, &reqestBuffers)) {
        perror("freeing buffers");
        return false;
    }
    state = (State)(state & ~REQBUFS);
    return true;
}
bool CameraDriver::newBufArray(){
    uint cMask = BUFFERS_ARRAY;
    if(state & cMask){
        cout << "buffers array already created" << endl;
        return false;
    }
    try{
        buffers = new v4l2_buffer[buffercount]();
    } catch(std::bad_alloc& err) {
        cout << "Cannot allocate buffers array: " << err.what() << endl;
        return false;
    }
    state = (State)(state | BUFFERS_ARRAY);
    return true;
}
bool CameraDriver::queryAllBuffers(){
    cout << "query all buffers" << endl;
    uint dMask = OPEN | REQBUFS | BUFFERS_ARRAY;
    if((state & dMask) != dMask){
        cout << "depends OPEN & REQBUFS & BUFFERS_ARRAY" << endl;
        return false;
    }else if(state & QUERYBUF){
        // can't double queryAll because it overwrites buffers array
        cout << "already queried" << endl;
        return false;
    }
    for (unsigned int i = 0; i < reqestBuffers.count; i++) {
        struct v4l2_buffer buffer;// = buffers[i];
        memset(&buffer, 0, sizeof(buffer));
        buffer.type = reqestBuffers.type;
        buffer.memory = reqestBuffers.memory;
        buffer.index = i;

        // Sets buffer.length and buffer.m.offset
        if (-1 == ioctl (fd, VIDIOC_QUERYBUF, &buffer)) {
            perror("did not get buffer parameters");
            return false;
        }
        // need info to mmap
        // this is the reason this action is needed
        buffers[i] = buffer;
    }
    state = (State)(state | QUERYBUF);
    return true;
}
bool CameraDriver::mMAP(){
    cout << "mmap" << endl;
    uint dMask = OPEN | REQBUFS | BUFFERS_ARRAY | QUERYBUF;
    uint cMask = MMAP | QBUF;
    if(! ((state & dMask) == dMask)){
        cout << "depends OPEN REQBUFS BUFFERS_ARRAY QUERYBUF" << endl;
        return false;
    } else if(state & cMask){
        cout << "MMAP already done" << endl;
        return false;
    }
    for (unsigned int i = 0; i < reqestBuffers.count; i++) {
        struct v4l2_buffer buffer = buffers[i];
        void * map = mmap(NULL,              // Kernel picks address
                     buffer.length,          // Driver dictates size
                     PROT_READ | PROT_WRITE, // Must match open flags
                     MAP_SHARED,             // Allows forking
                     fd,
                     buffer.m.offset);       // Driver dictated

        // use pointer field in buffer to store mmap pointer location
        // needed both for retrieving data, and freeing memory
        buffers[i].m.userptr = (long unsigned int)map;

        if (MAP_FAILED == map) {
            perror("mmap");
            for( ; i >= 1 ; i-- ){
                if (-1 == munmap((void *)buffers[i -1].m.userptr, buffers[i -1].length)){
                    perror("munmap");
                }
            }
//            delete[] buffers;
            return false;
        }
    }
    state = (State)(state | MMAP);
    return true;
}
bool CameraDriver::qbuf(){
    cout << "qbuf" << endl;
    uint dMask = OPEN | BUFFERS_ARRAY | MMAP;
    if(! ((state & dMask) == dMask)){
        cout << "depends OPEN BUFFERS_ARRAY MMAP" << endl;
        return false;
    } else if(state & STREAM){
        cout << "Don't do this while streaming is on" << endl;
        return false;
    }
    for (unsigned int i = 0; i < reqestBuffers.count; i++) {
        struct v4l2_buffer buffer = buffers[i];
        // Tell driver buffer is available for use
        if(-1 == ioctl(fd, VIDIOC_QBUF, &buffer)){
            perror("qbuf");
        }
    }
    state = (State)(state | QBUF);
    return true;
}
bool CameraDriver::freeMmap(){
    cout << "free mmap" << endl;
    uint dMask = OPEN | BUFFERS_ARRAY | MMAP;
    if(! ((state & dMask) == dMask)){
        cout << "depends OPEN BUFFERS_ARRAY MMAP" << endl;
        return false;
    } else if(state & STREAM){
        cout << "Cannot free mmap while streaming" << endl;
        return false;
    }
    bool errors = false;
    for (unsigned int i = 0; i < buffercount ; i++) {
        v4l2_buffer buffer = buffers[i];
        if (-1 == munmap((void *)buffer.m.userptr, buffer.length)){
            perror("munmap");
            errors = true;
        }
    }
    state = (State)(state & ~MMAP);
    return !errors;
}
bool CameraDriver::freeBufferArray(){
    uint dMask = OPEN | BUFFERS_ARRAY;
    uint cMask = MMAP;
    if(! ((state & dMask) == dMask)){
        cout << "depends OPEN BUFFERS_ARRAY" << endl;
        return false;
    } else if(state & cMask){
        cout << "cannot free buffer array while MMAP" << endl;
        return false;
    }
    delete[] buffers;
    state = (State)(state & ~(BUFFERS_ARRAY | QBUF | QUERYBUF));
    return true;
}
bool CameraDriver::startStream(){
    cout << "stream" << endl;
    uint dMask = OPEN | MMAP | QBUF;
    if(! ((state & dMask) == dMask)){
        cout << "depends OPEN MMAP QBUF" << endl;
        return false;
    } else if(state & STREAM){
        cout << "Already Streaming" << endl;
        return false;
    }
    if (-1 == ioctl(fd, VIDIOC_STREAMON, &reqestBuffers.type)){
        perror("start video stream");
        return false;
    }
    state = (State)(state | STREAM);
    return true;
}
bool CameraDriver::stopStream(){
    cout << "stream off" << endl;
    uint dMask = OPEN | STREAM;
    if(! ((state & dMask) == dMask)){
        cout << "depends OPEN STREAM" << endl;
        return false;
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(fd != -1){
        if (-1 == ioctl(fd, VIDIOC_STREAMOFF, &type)){
            perror("stop video stream");
            return false;
        }
    }
    state = (State)(state & ~STREAM);
    return true;
}

// return codes:
// 1 - OK
// 2 - no device
// 3 - wrong format
// 4 - idk
int CameraDriver::updateData(cv::Mat * depthmat, cv::Mat * irmat)
{
    uint dMask = OPEN | STREAM;
    if((state & dMask) != dMask){
        return 4;
    }
    struct v4l2_buffer dqbuf;
    memset(&dqbuf, 0, sizeof(dqbuf));
    dqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // cannot fetch buffer
    dqbuf.memory = V4L2_MEMORY_MMAP;          // without these fields set

    // get info of buffer that is oldest in queue "dequeue buffer"
    if (-1 == ioctl(fd, VIDIOC_DQBUF, &dqbuf)) {
       if(errno == EAGAIN){
           // frame not ready yet
       } else if(errno == ENODEV){
           return 2;
       } else {
           perror("readbuf");
           cout << "err!" << endl;
       }
    } else {
        // VIDIOC_DQBUF doesn't actually return pointer to data when using mmap
        // Structure has index that can be used to look up pointer
        void * voidData = (void *)buffers[dqbuf.index].m.userptr;
        // Do something with data
//        emit newData(data);
        //createImages(data);
        const int width = v4l2Format.fmt.pix.width;
        const int height = v4l2Format.fmt.pix.height;
        const u_int32_t pixelFormat = v4l2Format.fmt.pix.pixelformat;
        if(pixelFormat != INRI)
        {
            cout << "wrong call, scrub:" << pixelFormat << endl;
            ioctl(fd, VIDIOC_QBUF, &dqbuf);
            return 2;
        }

        Mat depth_cv_rgb(height,width,CV_8UC3);
        Mat ir_cv_rgb(height,width,CV_8UC3);


        u_int8_t * data = (u_int8_t *)voidData;

        Mat depth_cv(height,width, CV_16U);
        Mat ir_cv(height,width, CV_8U);
        Mat depth_cv_8;

        for(int j = 0 ; j < height ; j++){
            int step24 = width*3*j;
            for(int i = 0 ; i < width ; i++){
                int pixel24 = step24 + 3*i;
                u_int16_t depth = *(u_int16_t *)(data + pixel24);
                u_int8_t ir = data[pixel24 + 2];

                // removed because why the hell not
//                depth = int(depth/31.25 + 0.5); // convert to mm
//                u_int8_t high = (depth >> 8) & 0xff;
//                u_int8_t low = depth & 0xff;

//                Vec2b depthpix_cv;
//                depthpix_cv[0] = low;
//                depthpix_cv[1] = high;
//                depth_cv.at<cv::Vec2b>(j,i) = depthpix_cv;
                ir_cv.at<uchar>(j,i) = ir;
                depth_cv.at<ushort>(j,i) = depth;
//                if(i == 100 && j == 100)
//                {
//                    std::cout << "depth" << depth << std::endl;
//                    std::cout << "depth2" << depth_cv.at<ushort>(i,j) << std::endl;
//                }
            }
        }
//        depth_cv.convertTo(depth_cv_8,CV_8U,1.0/256.0);
//        cvtColor(depth_cv_8,depth_cv_rgb,CV_GRAY2RGB);
//        cvtColor(ir_cv,ir_cv_rgb,CV_GRAY2RGB);

        *depthmat = depth_cv;
        *irmat = ir_cv;

        // tell driver it can reuse framebuffer
        ioctl(fd, VIDIOC_QBUF, &dqbuf);
        // signal repaint();
        return 1;
    }
}
int CameraDriver::updateData(cv::Mat * rgb)
{
    uint dMask = OPEN | STREAM;
    if((state & dMask) != dMask){
        return 4;
    }
    struct v4l2_buffer dqbuf;
    memset(&dqbuf, 0, sizeof(dqbuf));
    dqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // cannot fetch buffer
    dqbuf.memory = V4L2_MEMORY_MMAP;          // without these fields set

    // get info of buffer that is oldest in queue "dequeue buffer"
    if (-1 == ioctl(fd, VIDIOC_DQBUF, &dqbuf)) {
       if(errno == EAGAIN){
           // frame not ready yet
       } else if(errno == ENODEV){
           return 2;
       } else {
           perror("readbuf");
           cout << "err!" << endl;
       }
    } else {
        // VIDIOC_DQBUF doesn't actually return pointer to data when using mmap
        // Structure has index that can be used to look up pointer
        void * dataV = (void *)buffers[dqbuf.index].m.userptr;
        // Do something with data
//        emit newData(data);

        // inline fucking everything
        //createImages(data);
        const int width = v4l2Format.fmt.pix.width;
        const int height = v4l2Format.fmt.pix.height;
        const u_int32_t pixelFormat = v4l2Format.fmt.pix.pixelformat;
        if(pixelFormat != YUYV)
        {
            cout << "wrong call, scrub" << endl;
            ioctl(fd, VIDIOC_QBUF, &dqbuf);
            return 2;
        }

        Mat color_cv_rgb;

        u_int8_t * data = (u_int8_t *)dataV;
        Mat color_cv;
        //Mat color_cv_rgb;

        color_cv = Mat(height,width, CV_8UC2, data);
        cvtColor(color_cv,color_cv_rgb,CV_YUV2RGB_YUYV);

        *rgb = color_cv_rgb;

        //namedWindow( "Display window driver", WINDOW_AUTOSIZE );// Create a window for display.
        //imshow( "Display window driver", color_cv_rgb );                   // Show our image inside it.

        //waitKey(0);

        // tell driver it can reuse framebuffer
        ioctl(fd, VIDIOC_QBUF, &dqbuf);
        // signal repaint();
        return 1;
    }
}
//void CameraDriver::createImages(void * voidData){
//    const int width = v4l2Format.fmt.pix.width;
//    const int height = v4l2Format.fmt.pix.height;
//    const u_int32_t pixelFormat = v4l2Format.fmt.pix.pixelformat;
//    u_int8_t * data = (u_int8_t *)voidData;

//    Mat color_cv; //(height,width,CV_8UC2);
//    Mat color_cv_rgb;
//    Mat depth_cv(height,width, CV_16U);
//    Mat ir_cv(height,width, CV_8U);
//    Mat depth_cv_8;
//    Mat depth_cv_rgb(height,width,CV_8UC3);
//    Mat ir_cv_rgb(height,width,CV_8UC3);

//    switch(pixelFormat){
//    case YUYV:
//        color_cv = Mat(height,width, CV_8UC2, data);
//        cvtColor(color_cv,color_cv_rgb,CV_YUV2RGB_YUYV);
//        colorImage = QImage(color_cv_rgb.data,color_cv_rgb.cols,color_cv_rgb.rows,
//                            color_cv_rgb.step,QImage::Format_RGB888).copy();
//        emit newColorImage(colorImage);
//        break;
//    case INVZ:
//    case INVR:
//        // process depth
//        break;
//    case INVI:
//    case RELI:
//        // process infrared
//        break;
//    case INZI:
//    case INRI:
//        //process depth/ir
//        for(int j = 0 ; j < height ; j++){
//            int step24 = width*3*j;
//            for(int i = 0 ; i < width ; i++){
//                int pixel24 = step24 + 3*i;
//                u_int16_t depth = *(u_int16_t *)(data + pixel24);
//                u_int8_t ir = data[pixel24 + 2];
////                depth = int(depth/31.25 + 0.5); // convert to mm
//                u_int8_t high = (depth >> 8) & 0xff;
//                u_int8_t low = depth & 0xff;
//                Vec2b depthpix_cv;
//                depthpix_cv[0] = low;
//                depthpix_cv[1] = high;
//                depth_cv.at<cv::Vec2b>(j,i) = depthpix_cv;
//                ir_cv.at<uchar>(j,i) = ir;
//            }
//        }
//        depth_cv.convertTo(depth_cv_8,CV_8U,1.0/256.0);
//        cvtColor(depth_cv_8,depth_cv_rgb,CV_GRAY2RGB);
//        depthImage = QImage(depth_cv_rgb.data,depth_cv_rgb.cols,depth_cv_rgb.rows,
//                            depth_cv_rgb.step,QImage::Format_RGB888).copy();
//        emit newDepthImage(depthImage);
//        cvtColor(ir_cv,ir_cv_rgb,CV_GRAY2RGB);
//        infraredImage = QImage(ir_cv_rgb.data,ir_cv_rgb.cols,ir_cv_rgb.rows,
//                            ir_cv_rgb.step,QImage::Format_RGB888).copy();
//        emit newInfraredImage(infraredImage);

//        break;
//    }
//}
