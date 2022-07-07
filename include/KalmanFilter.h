#ifndef _KALMAN_FILTER_
#define _KALMAN_FILTER_

#include "opencv2/video/tracking.hpp"

class KalmannFilter 
{
public:

	KalmanFilter();

	KalmanFilter(cv::Rect initRect);

	~KalmanFilter();

	cv::Rect2d predict();

	cv::Rect2d curState();

	void update(cv::Rect);

	int m_lastUpdate;
	int m_lastPredict;

private:

	void initialize(cv::Rect);

	cv::KalmanFilter m_kf;

	cv::Mat m_meas;

	std::vector<cv::Rect> m_history;

	static int m_counter;
};

#endif
