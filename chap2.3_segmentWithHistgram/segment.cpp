/*
 ==============================

 ** 3D视觉在缺陷检测的应用
 ** 第二章： 2D图像处理基础
 ** 3D视觉工坊
 ** 林子祥

** 任务：基于直方图统计分析的深度图分割，将目标元器件区域提取出来

==============================
*/


#include <iostream>

//! Add assign head file
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <matplotlibcpp.h> 

cv::Mat calcGrayHist(const cv::Mat & image)
{
    cv::Mat hist = cv::Mat::zeros(cv::Size(256, 1), CV_32SC1);
    int rows = image.rows;
    int cols = image.cols;
    int index = 0;
    for (int r = 0; r < rows; r++)  {
        for (int c = 0; c < cols; c++) {
            index = static_cast<int>(image.at<uchar>(r, c));
            hist.ptr<int>(0)[index] += 1;
        }
    }
    cv:
    return hist;
}

// Compute Otsu threshold for segmentation
int otsuThreshold(const cv::Mat& image)
{
    // Compute histogram
    cv::Mat hist = calcGrayHist(image);

    // Normalize histogram
    cv::Mat normHist;
    hist.convertTo(normHist, CV_32FC1, 1.0 / (image.total()), 0.0);

    // Calculate the cumulative histogram (zero-order cumulative moment) and the first-order cumulative moment
    cv::Mat zeroCumuMomnet = cv::Mat::zeros(cv::Size(256,1), CV_32FC1);
    cv::Mat oneCumuMoment = cv::Mat::zeros(cv::Size(256,1), CV_32FC1);
    for(int i = 0; i < 256; i++) {
        if(i == 0)  {
            zeroCumuMomnet.ptr<float>(0)[i] = normHist.ptr<float>(0)[i];
            oneCumuMoment.ptr<float>(0)[i] = i * normHist.ptr<float>(0)[i];
        } else {
            zeroCumuMomnet.ptr<float>(0)[i] = zeroCumuMomnet.ptr<float>(0)[i-1] + normHist.ptr<float>(0)[i];
            oneCumuMoment.ptr<float>(0)[i] = oneCumuMoment.ptr<float>(0)[i-1] + i * normHist.ptr<float>(0)[i];
        }
    }

    // Calculate the variance between classes
    int thresh = 0;
    float variance = 0, varianceTmp;
    float mean = oneCumuMoment.at<float>(0, 255);
    for (int i = 0; i < 256; i++)  {
        if (zeroCumuMomnet.ptr<float>(0)[i] == 0 || zeroCumuMomnet.ptr<float>(0)[i] == 1) {
            varianceTmp = 0;
        } else  {
            float cofficient = zeroCumuMomnet.ptr<float>(0)[i] * (1.0-zeroCumuMomnet.ptr<float>(0)[i]);
            varianceTmp = pow(mean*zeroCumuMomnet.ptr<float>(0)[i] - oneCumuMoment.ptr<float>(0)[i], 2.0) / cofficient;
        } 
        if (variance < varianceTmp)  {
            thresh = i;
            variance = varianceTmp;
        }
    }

    return thresh;
}

void print_img_element(cv::Mat img){

    // Print out the element of the image

    std::cout << "Depth channel size =" << (double)img.channels() << std::endl;
    int val_min = 0;
    int val_max = 0;
    for(int r=0; r<img.rows; r++){
        for(int c=0; c<img.cols; c++){
            if (((float)img.at<uchar>(r,c)) < val_min){
                val_min = (float)img.at<uchar>(r,c);
            };
            if (((float)img.at<uchar>(r,c)) >= val_max){
                val_max = (float)img.at<uchar>(r,c);
            };
        }
    }
    std::cout << "Type: " << typeid(img).name() << std::endl;
    std::cout << "Type2: " << img.type() << std::endl;
    std::cout << "Value max = " << val_max << std::endl;
    std::cout << "Value min = " << val_min << std::endl;

}

void plot_hist(cv::Mat img){

    // Calculate histogram
    cv::Mat hist;
    int histSize = 256;   
    float range[] = {0, 256};
    const float* histRange = {range};
    cv::calcHist(&img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

    // Plot histogram 
    int hist_w = 512; 
    int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );

    cv::Mat histImage(hist_h, hist_w, CV_8UC1, cv::Scalar(0,0,0));

    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    for( int i = 1; i < histSize; i++ ) {
    cv::line(histImage, cv::Point(bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1))),  
            cv::Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
            cv::Scalar(255, 0, 0), 2, 8, 0  );
    }

    cv::imshow("Histogram", histImage);
    cv::waitKey(0);

}

int main(int argc, char*argv[])
{
    // Load depth image
    cv::Mat depth = cv::imread("../depthMap.png", cv::IMREAD_GRAYSCALE);

    if(depth.empty()) {
        std::cout << "Could not load depth image" << std::endl;
        return -1;
    }

    // Plot the histogram of the image
    plot_hist(depth);

    // Compute Otsu threshold
    int thresh = otsuThreshold(depth);
    std::cout << "Threshold = " << thresh << std::endl;

    // Apply threshold (obtained from otsu threshold) to get mask
    cv::Mat mask;
    cv::threshold(depth, mask, thresh, 255, cv::THRESH_BINARY);

    // Print each element
    print_img_element(depth);

    // Display result
    cv::imshow("Depth Mask", depth);
    cv::waitKey(0);

    return 0;
}
