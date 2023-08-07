#include <opencv2/imgproc.hpp>  // Image processing
#include <opencv2/highgui.hpp>  // Image I/O and GUI

#include <iostream> // For console I/O

cv::Mat mean_filter(cv::Mat src){

  // Create 3x3 mean filter kernel 
  cv::Mat mean_kernel = (cv::Mat_<double>(3,3) << 1, 1, 1, 
                                                  1, 1, 1, 
                                                  1, 1, 1) / 9;
  
  // Apply mean filter on source image
  cv::Mat dst;
  cv::filter2D(src, dst, -1, mean_kernel);

  return dst;

}

cv::Mat median_filter(cv::Mat src){
  
  cv::Mat dst;

  // Apply median filter on source image with kernel size 3
  cv::medianBlur(src, dst, 3); 

  return dst;

}

cv::Mat gaussian_filter(cv::Mat src){

  cv::Mat dst;

  //Blur the image with 3x3 Gaussian kernel
  cv::GaussianBlur(src, dst, cv::Size(3, 3), 0);

  return dst;

}

int main(int argc, char** argv) {

  // Check command line arguments
  if (argc != 2){
    std::cout << "Usage: " << argv[0] << " <image_file_path>" << std::endl;
    return -1;
  }

  // Get the image file path
  // argv[0] is the compiled file which is "main"
  std::string img_file = argv[1];
  // std::string img_file = "../test.jpeg";

  // Read image
  cv::Mat src = cv::imread(img_file, cv::IMREAD_COLOR);

  // Check if image loaded fine
  if (src.empty()){
      std::cout << "Error loading image file: " << img_file << std::endl;
      return -1;
  }
  
  // Apply 3 types of filtering technique (mean, median and gaussian)
  cv::Mat dst1 = mean_filter(src);
  cv::Mat dst2 = median_filter(src);
  cv::Mat dst3 = gaussian_filter(src);

  // Display input and output images
  cv::imshow("Original Image", src);
  cv::imshow("Mean Filtered Image", dst1);
  cv::imshow("Median Filtered Image", dst2);
  cv::imshow("Gaussian Filtered Image", dst3);

  // Wait until any key is pressed
  cv::waitKey(0);

  // Release window resources
  cv::destroyAllWindows();
  
  return 0;
}