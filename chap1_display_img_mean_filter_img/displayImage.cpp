#include <opencv2/imgproc.hpp>  // Image processing
#include <opencv2/highgui.hpp>  // Image I/O and GUI

int main()
{
	
	// Read image
	cv::Mat img = cv::imread("../test.jpeg", cv::IMREAD_COLOR);
	// cv::Mat img = cv::Mat::zeros(100, 100, CV_8U);

  	// Display input and output images
	cv::imshow("img", img);
	
  	// Wait until any key is pressed
	cv::waitKey(0);

  	// Release window resources
	cv::destroyAllWindows();
	
	return 0;
}

