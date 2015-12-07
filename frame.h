#ifndef FRAME_H
#define FRAME_H


class Frame
{
public:
    Frame(std::vector<cv::Mat> rawStackDepth, cv::Mat cameraMatrix); // add converter function pointer
    //void setConverter();
    //gettransform()
    //settransform
    void convert();
    cv::Mat getImage();
    pcl::PointXYZ getCloud();
private:
    cv::Mat cameraMatrix;
    //cv::Mat?? Transform
    cv::Mat belief;
    cv::Mat processedImageDepth;
    std::vector<cv::Mat> rawStackIR;
    std::vector<cv::Mat> rawStackDepth;
    pcl::PointXYZ cloud;
};

#endif // FRAME_H
