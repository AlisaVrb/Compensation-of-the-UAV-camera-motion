#ifndef _KALMAN_FILTER_
#define _KALMAN_FILTER_

#ifndef ALISA_CLS_EXPORTS
# if (defined _WIN32 || defined _MSC_VER)
#   define ALISA_CLS_EXPORTS __declspec(dllexport)
# elif defined __GNUC__ && __GNUC__ >= 7 
#   define ALISA_CLS_EXPORTS __attribute__ ((visibility ("default")))
# else 
#	define ALISA_CLS_EXPORTS
# endif
#endif

#include "opencv2/video/tracking.hpp"

class ALISA_CLS_EXPORTS KalmanFilter
{
public:

	KalmanFilter();

	KalmanFilter(cv::Rect initRect);

	~KalmanFilter() =default;

	cv::Rect predict();

	cv::Rect curState();

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
