#ifndef CAMERAMOTIONESTIMATION_H
#define CAMERAMOTIONESTIMATION_H

class CameraMotionEstimator
{
private:

	cv::Mat img_cur; // current frame

	std::vector<cv::Point2f> p0, p1, p0_check; // p0 - previous points, p1 - current points

	cv::Mat gray_firstframe;

	cv::Mat H;
	
public:
	
	void Find_Shi_Tomasi_corners(cv::Mat first_frame, int maxCorners = 50, double qualityLevel = 0.04,
		double minDistance = 5, int blockSize = 3);
	
	void OpticalFlow(cv::Mat frame);
	
	void Homography(int Min_Match = 10);
	
}; 
#endif 
