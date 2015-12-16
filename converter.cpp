#include "converter.h"

Converter::Converter()
{

}

std::string Converter::type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

// TODO: add implementation for IR, check type
void Converter::analyseStack(std::vector<cv::Mat> &stack, cv::Mat & believe, cv::Mat & result)
{
    for (int i = 0; i < stack[0].rows; i++)
    {
        for (int j = 0; j < stack[0].cols; j++)
        {
            unsigned long tempresult_depth = 0;
            int depth_zeroes = 0;
            for(int il = 0; il < stack.size(); il++)
            {
                tempresult_depth += (int)stack[il].at<ushort>(i,j);
                if((int)stack[il].at<ushort>(i,j) == 0)
                    depth_zeroes++;
            }
            // TODO: filter von unzuverlässigen daten. möglicherweise in neue methode auslagern
            if(depth_zeroes < stack.size()-0)
                result.at<ushort>(i,j) = tempresult_depth/(stack.size()-depth_zeroes);
            else
                result.at<ushort>(i,j) = 0;
            believe.at<uchar>(i,j) = (int)((240/stack.size())*depth_zeroes);
        }
    }

//    for (int i = 0; i < irlist[0].rows; i++)
//    {
//        for (int j = 0; j < irlist[0].cols; j++)
//        {
//            int tempresult_ir = 0;
//            int tempresult_depth = 0;
//            int depth_zeroes = 0;
//            for(int il = 0; il < irlist.size(); il++) {
//                tempresult_ir += (int)irlist[il].at<uchar>(i,j);
//                tempresult_depth += (int)depthlist[il].at<ushort>(i,j);
//                if((int)depthlist[il].at<ushort>(i,j) == 0)
//                    depth_zeroes++;
//            }
//            result_ir.at<uchar>(i,j) = tempresult_ir/irlist.size();
//            // depthlist.size() - x # alle mit weniger als x falschen pixeln
//            if(depth_zeroes < depthlist.size())
//                result_depth.at<ushort>(i,j) = tempresult_depth/(depthlist.size()-depth_zeroes);
//            else
//                result_depth.at<ushort>(i,j) = 0;
//            result_zeroes.at<uchar>(i,j) = (int)((240/irlist.size())*depth_zeroes);
//        }
    //    }
}


void Converter::undistortDepth(cv::Mat depth)
{
    // correct depth
    // fancy überlegung von alex und peter
    for(int x = 0; x<640; x++)
    {
        for(int y = 0; y<480; y++)
        {
            int z = (int)depth.at<ushort>(y,x);
            // calculate alpha, x axis
            double tanx = (0.44*fabs(x-328.001)) / 205;
            double atanx = atan(tanx); // (result in radians)
            // calculate side b, x axis
            double sideb = cos(atanx) * z;

            // calculate alpha, y axis
            tanx = (0.44*fabs(y-242.001)) / 205;
            atanx = atan(tanx); // (result in radians)
            // calculate side b, y axis
            sideb = cos(atanx) * sideb;

            depth.at<ushort>(y,x) = sideb;

        }
    }
}

/// stolen from opencv contribution
void Converter::rescaleDepth(cv::InputArray in_in, int depth, cv::OutputArray out_out)
{
    cv::Mat in = in_in.getMat();
    CV_Assert(in.type() == CV_64FC1 || in.type() == CV_32FC1 || in.type() == CV_16UC1 || in.type() == CV_16SC1);
    CV_Assert(depth == CV_64FC1 || depth == CV_32FC1);

    int in_depth = in.depth();

    out_out.create(in.size(), depth);
    cv::Mat out = out_out.getMat();
    if (in_depth == CV_16U)
    {
        in.convertTo(out, depth, 1 / 1000.0); //convert to float so that it is in meters
        cv::Mat valid_mask = in == std::numeric_limits<ushort>::min(); // Should we do std::numeric_limits<ushort>::max() too ?
        out.setTo(std::numeric_limits<float>::quiet_NaN(), valid_mask); //set a$
    }
    if (in_depth == CV_16S)
    {
        in.convertTo(out, depth, 1 / 1000.0); //convert to float so tha$
        cv::Mat valid_mask = (in == std::numeric_limits<short>::min()) | (in == std::numeric_limits<short>::max()); // Should we do std::numeric_limits<ushort>::max() too ?
        out.setTo(std::numeric_limits<float>::quiet_NaN(), valid_mask); //set a$
    }
    if ((in_depth == CV_32F) || (in_depth == CV_64F))
        in.convertTo(out, depth);
}

/// stolen (and modified) from opencv contribution
void Converter::depthTo3d(const cv::Mat& in_depth, const cv::Mat& K, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
    pcl::PointXYZ newPoint;

    const double inv_fx = double(1) / K.at<double>(0, 0);
    const double inv_fy = double(1) / K.at<double>(1, 1);
    const double ox = K.at<double>(0, 2);
    const double oy = K.at<double>(1, 2);

    // Build z
    cv::Mat_<double> z_mat;
    if (z_mat.depth() == in_depth.depth())
        z_mat = in_depth;
    else
        rescaleDepth(in_depth, CV_64F, z_mat);



    // Pre-compute some constants
    cv::Mat_<double> x_cache(1, in_depth.cols), y_cache(in_depth.rows, 1);
    double* x_cache_ptr = x_cache[0], *y_cache_ptr = y_cache[0];
    for (int x = 0; x < in_depth.cols; ++x, ++x_cache_ptr)
        *x_cache_ptr = (x - ox) * inv_fx;
    for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
        *y_cache_ptr = (y - oy) * inv_fy;

    y_cache_ptr = y_cache[0];


    for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
    {
        const double* x_cache_ptr_end = x_cache[0] + in_depth.cols;
        const double* depth = z_mat[y];
        for (x_cache_ptr = x_cache[0]; x_cache_ptr != x_cache_ptr_end; ++x_cache_ptr, ++depth)
        {
            double z = *depth;
            if(isnan(z))
            {
                newPoint.z = std::numeric_limits<float>::quiet_NaN();
                newPoint.x = std::numeric_limits<float>::quiet_NaN();
                newPoint.y = std::numeric_limits<float>::quiet_NaN();
                cloud->push_back(newPoint);
            }else
            {
                newPoint.z = z;
                newPoint.x = (*x_cache_ptr) * z * -1.0;
                newPoint.y = (*y_cache_ptr) * z * -1.0;
                cloud->push_back(newPoint);
            }

        }
    }
}

void Converter::averageIR(const std::vector<cv::Mat> & IRstack, cv::Mat & ir_avg)
{
    for (int i = 0; i < IRstack[0].rows; i++)
    {
        for (int j = 0; j < IRstack[0].cols; j++)
        {
            long tempresult_ir = 0;
            for(int il = 0; il < IRstack.size(); il++) {
                tempresult_ir += (int)IRstack[il].at<uchar>(i,j);
            }
            ir_avg.at<uchar>(i,j) = tempresult_ir/IRstack.size();
        }
    }
}
