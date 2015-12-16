#include "tracker.h"

Tracker::Tracker()
{
    TheIntrinsicFile = "intrinsicsrealsense.yml";
    TheBoardConfigFile = "chessinfo_meter.yml";
    TheMarkerSize = 0.0325;
    conversionmultiplier = 100.0; // alt. 2.4

    TheBoardConfig.readFromFile(TheBoardConfigFile);
    TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
    TheCameraParameters.resize(cv::Size(640,480));

    ThresParam1 = 13.0;
    ThresParam2 = 8.0;
    iThresParam1 = 13;
    iThresParam2 = 8;

    TheBoardDetector.setParams(TheBoardConfig, TheCameraParameters, TheMarkerSize);
    TheBoardDetector.getMarkerDetector().getThresholdParams(ThresParam1, ThresParam2);
    TheBoardDetector.getMarkerDetector().setCornerRefinementMethod(aruco::MarkerDetector::HARRIS);
    TheBoardDetector.set_repj_err_thres(1.5);
    // 	TheBoardDetector.getMarkerDetector().enableErosion(true);//for chessboards


    cv::namedWindow("thres", 1);
    cv::namedWindow("in", 1);

    iThresParam1 = ThresParam1;
    iThresParam2 = ThresParam2;
    cv::createTrackbar("ThresParam1", "in", &iThresParam1, 13, cvTackBarEvents, this);
    cv::createTrackbar("ThresParam2", "in", &iThresParam2, 13, cvTackBarEvents, this);
}

void Tracker::getTransformation(const cv::Mat &image, Eigen::Affine3f &Matrix)
{
    getTransformAruCoChessboard(image, Matrix);
}

Eigen::Affine3f Tracker::create_rotation_matrix(double ax, double ay, double az)
{
    Eigen::Affine3f rx =
            Eigen::Affine3f(Eigen::AngleAxisf(ax, Eigen::Vector3f(1, 0, 0)));
    Eigen::Affine3f ry =
            Eigen::Affine3f(Eigen::AngleAxisf(ay, Eigen::Vector3f(0, 1, 0)));
    Eigen::Affine3f rz =
            Eigen::Affine3f(Eigen::AngleAxisf(az, Eigen::Vector3f(0, 0, 1)));
    return rz * ry * rx;
}

//void Tracker::getTransformOpenCVChessboard(const cv::Mat &image, Eigen::Affine3f &Matrix)
//{
//    // fancy chessboard shit
//    // code stolen from: https://github.com/foxymop/3DPoseEstimation/blob/master/src/coordinate_system.cpp
//    //vector<Point2f> pointBuf;
//    //cv::findChessboardCorners(fr.getIRImage(),cv::Size(8,6),pointBuf,cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
//    //cv::cornerSubPix(fr.getIRImage(),pointBuf,cv::Size(11,11), cv::Size(-1,-1),cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 30, 0.1 ));

//    cv::Mat testimage = fr.getIRImage();
//    //testimage = disp.rectify(cw.cameraMatrix, cw.distCoeffs, testimage);

//    int boardHeight = 7;
//    int boardWidth = 10;
//    Size cbSize = Size(boardHeight,boardWidth);
//    Mat webcamImage, gray, one;
//    Mat rvec = Mat(Size(3,1), CV_64F);
//    Mat tvec = Mat(Size(3,1), CV_64F);
//    //setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
//    vector<Point2d> imagePoints, imageFramePoints, imageOrigin;
//    vector<Point3d> boardPoints, framePoints;

//    for (int i=0; i<boardWidth; i++)
//    {
//        for (int j=0; j<boardHeight; j++)
//        {
//            boardPoints.push_back( Point3d( double(i), double(j), 0.0) );
//        }
//    }
//    //generate points in the reference frame
//    framePoints.push_back( Point3d( 0.0, 0.0, 0.0 ) );
//    framePoints.push_back( Point3d( 5.0, 0.0, 0.0 ) );
//    framePoints.push_back( Point3d( 0.0, 5.0, 0.0 ) );
//    framePoints.push_back( Point3d( 0.0, 0.0, 5.0 ) );


//    bool found = findChessboardCorners(testimage, cbSize, imagePoints, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

//    if ( found )
//    {
//        //find the camera extrinsic parameters
//        solvePnP( Mat(boardPoints), Mat(imagePoints), cw.cameraMatrix, cw.distCoeffs, rvec, tvec, false );

//        //project the reference frame onto the image
//        projectPoints(framePoints, rvec, tvec, cw.cameraMatrix, cw.distCoeffs, imageFramePoints );


//        //DRAWING
//        //draw the reference frame on the image
//        circle(testimage, (cv::Point2d) imagePoints[0], 4 ,CV_RGB(255,0,0) );

//        Point one, two, three;
//        one.x=10; one.y=10;
//        two.x = 60; two.y = 10;
//        three.x = 10; three.y = 60;

//        line(testimage, one, two, CV_RGB(255,0,0) );
//        line(testimage, one, three, CV_RGB(0,255,0) );

//        line(testimage, imageFramePoints[0], imageFramePoints[1], CV_RGB(255,0,0), 2 );
//        line(testimage, imageFramePoints[0], imageFramePoints[2], CV_RGB(0,255,0), 2 );
//        line(testimage, imageFramePoints[0], imageFramePoints[3], CV_RGB(0,0,255), 2 );

//        //show the pose estimation data
//        cout << fixed << setprecision(2) << "rvec = ["
//             << rvec.at<double>(0,0) << ", "
//             << rvec.at<double>(1,0) << ", "
//             << rvec.at<double>(2,0) << "] \t" << "tvec = ["
//             << tvec.at<double>(0,0) << ", "
//             << tvec.at<double>(1,0) << ", "
//             << tvec.at<double>(2,0) << "]" << endl;

//    }

//    namedWindow( "testimage", cv::WINDOW_AUTOSIZE );
//    cv::imshow( "testimage", testimage);
//}


/// Aruco
/// http://www.uco.es/investiga/grupos/ava/node/26
/// http://sourceforge.net/projects/aruco/files/?source=navbar
///
/// Input: image (cv::Mat)
/// Output: Transformation Matrix (Eigen::Affine3f)
///
/// Trackbars to modify treshold parameters
/// detects pattern and computes transformation of markerboard in camera coordinate system
/// If the board is not detected (prob < 0.2 - whatever that means) Identity matrix is returned.
void Tracker::getTransformAruCoChessboard(const cv::Mat &image, Eigen::Affine3f &Matrix)
{
    cv::Mat TheInputImage, TheInputImageCopy;
    image.copyTo(TheInputImage);
    //image.copyTo(TheInputImageCopy);

    // detect marker
    float probDetect = TheBoardDetector.detect(image);

    // draw detected markers
    image.copyTo(TheInputImageCopy);
    for (unsigned int i = 0; i < TheBoardDetector.getDetectedMarkers().size(); i++)
        TheBoardDetector.getDetectedMarkers()[i].draw(TheInputImageCopy, cv::Scalar(0, 0, 255), 1);

    // draw board axis
    if (TheCameraParameters.isValid()) {
        if (probDetect > 0.2) {
            aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheBoardDetector.getDetectedBoard(), TheCameraParameters);
            // 		    CvDrawingUtils::draw3dCube(TheInputImageCopy, TheBoardDetector.getDetectedBoard(),TheCameraParameters);
            // draw3dBoardCube( TheInputImageCopy,TheBoardDetected,TheIntriscCameraMatrix,TheDistorsionCameraParams);
        }
    }

    if (probDetect > 0.2){
        aruco::Board detBoard = TheBoardDetector.getDetectedBoard();
//        std::cout << detBoard.Rvec << std::endl << detBoard.Tvec << std::endl;
        Matrix = createMatrixfromVectors(detBoard.Rvec, detBoard.Tvec);

    }else
        Matrix = Eigen::Affine3f::Identity();

    cv::imshow("in", TheInputImageCopy);
    cv::imshow("thres", TheBoardDetector.getMarkerDetector().getThresholdedImage());
    //cv::waitKey(0);//wait for key to be pressed
}


/// callback in object: http://opencv-users.1802565.n2.nabble.com/Member-function-callback-in-cvCreateTrackbar-td3785481.html
///                     http://stackoverflow.com/questions/8636689/opencv-trackbar-callback-in-c-class
void Tracker::cvTackBarEvents(int pos, void *obj) {
    Tracker* myObj = (Tracker*) obj;
    if (myObj->iThresParam1 < 3)
        myObj->iThresParam1 = 3;
    if (myObj->iThresParam1 % 2 != 1)
        myObj->iThresParam1++;
    if (myObj->ThresParam2 < 1)
        myObj->ThresParam2 = 1;
    myObj->ThresParam1 = myObj->iThresParam1;
    myObj->ThresParam2 = myObj->iThresParam2;
    myObj->TheBoardDetector.getMarkerDetector().setThresholdParams(myObj->ThresParam1, myObj->ThresParam2);
}

Eigen::Affine3f Tracker::createMatrixfromVectors(const cv::Mat &rvec, const cv::Mat &tvec)
{
    /// http://www.cplusplus.com/reference/iomanip/setprecision/
    std::cout << std::fixed << std::setprecision(2) << "rvec = ["
         << rvec.at<float>(0,0) << ", "
         << rvec.at<float>(1,0) << ", "
         << rvec.at<float>(2,0) << "] \t" << "tvec = ["
         << tvec.at<float>(0,0) << ", "
         << tvec.at<float>(1,0) << ", "
         << tvec.at<float>(2,0) << "]" << std::endl;

    /// http://pointclouds.org/documentation/tutorials/matrix_transform.php
    /// http://stackoverflow.com/questions/12933284/rodrigues-into-eulerangles-and-vice-versa
    Eigen::Affine3f mpose = Eigen::Affine3f::Identity();
    mpose.translation() << tvec.at<float>(0,0)*(-1*conversionmultiplier), tvec.at<float>(1,0)*(-1*conversionmultiplier), tvec.at<float>(2,0)*conversionmultiplier;
    float theta = sqrt(pow(rvec.at<float>(0,0),2) + pow(rvec.at<float>(1,0),2) + pow(rvec.at<float>(2,0),2));
    mpose.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f(rvec.at<float>(0,0)/theta*-1, rvec.at<float>(1,0)/theta*-1, rvec.at<float>(2,0)/theta)));

    //        /// http://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix
    //        Eigen::Affine3f r = create_rotation_matrix(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0));
    //        Eigen::Affine3f markerpose(Eigen::Translation3f(Eigen::Vector3f((tvec.at<double>(0,0)*-2.4),(tvec.at<double>(1,0)*-2.4),tvec.at<double>(2,0)*2.4)));
    //        markerpose = markerpose*r;

    return mpose;

}
