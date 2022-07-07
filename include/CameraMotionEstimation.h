#ifndef CAMERAMOTIONESTIMATION_H
#define CAMERAMOTIONESTIMATION_H

#ifndef ALISA_CLS_EXPORTS
# if (defined _WIN32 || defined _MSC_VER)
#   define ALISA_CLS_EXPORTS __declspec(dllexport)
# elif defined __GNUC__ && __GNUC__ >= 7 
#   define ALISA_CLS_EXPORTS __attribute__ ((visibility ("default")))
# else 
#	define ALISA_CLS_EXPORTS
# endif
#endif

#include <opencv2/opencv.hpp>

class ALISA_CLS_EXPORTS CameraMotionEstimator
{
public:
	
	void findShiTomasiCorners(cv::Mat first_frame, int maxCorners = 50, double qualityLevel = 0.04, double minDistance = 5, int blockSize = 3);
	
	void evalOpticalFlow(cv::Mat frame);
	
	void evalHomography(int Min_Match = 10);

private:

  cv::Mat m_cimg; // current frame
  cv::Mat m_pimg;
	cv::Mat m_first;
	cv::Mat m_H;

	std::vector<cv::Point2f> m_p0;
	std::vector<cv::Point2f> m_p1;
	std::vector<cv::Point2f> m_chkp0; // p0 - previous points, p1 - current points
}; 
#endif 
