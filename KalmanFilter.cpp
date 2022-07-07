#include "/Users/user/source/repos/KalmanFilter.h"

class KalmannFilter {
private:

	const std::size_t STATE_SZ = 7;
	const std::size_t ESIMATE_SZ = 4;
	const std::size_t ESTIMATE_COLS = 1;

	const double PROCESS_VAR = 1E-2;
	const double MEASUREMENTS_VAR = 1E-1;
	const double ESTIMATIONS_VAR = 1E+0;

	cv::KalmanFilter m_kf;
	std::vector<cv::Rect> m_history;
	static int m_counter;
	int m_lastUpdate;
	int m_lastPredict;
	std::string m_id;
	cv::Mat m_meas;

public:
	void initialize(cv::Rect rect)
	{
		m_kf = cv::KalmanFilter(STATE_SZ, ESIMATE_SZ, 0, CV_64F);
		m_meas = cv::Mat::zeros(ESIMATE_SZ, ESTIMATE_COLS, CV_64FC1);
		m_kf.transitionMatrix = (cv::Mat_<double>(STATE_SZ, STATE_SZ) <<
			1., 0., 0., 0.,
			0., 1., 0., 0.,
			0., 0., 1., 0.,
			0., 0., 0., 1.);;


		setIdentity(m_kf.measurementMatrix);
		setIdentity(m_kf.processNoiseCov, cv::Scalar::all(PROCESS_VAR));
		setIdentity(m_kf.measurementNoiseCov, cv::Scalar::all(MEASUREMENTS_VAR));
		setIdentity(m_kf.errorCovPost, cv::Scalar::all(ESTIMATIONS_VAR));

		m_kf.statePost.at<double>(0, 0) = rect.x + rect.width / 2.;
		m_kf.statePost.at<double>(1, 0) = rect.y + rect.height / 2.;
		m_kf.statePost.at<double>(2, 0) = rect.area();
		m_kf.statePost.at<double>(3, 0) = double(rect.width) / double(rect.height);
	}

	cv::Rect invRectTransform(double cx, double cy, double s, double r)
	{
		double w = sqrt(s * r);
		double h = s / w;
		double x = (cx - w / 2);
		double y = (cy - h / 2);

		if (x < 0 && cx > 0)
			x = 0;
		if (y < 0 && cy > 0)
			y = 0;

		return cv::Rect(x, y, w, h);
	}

	void KalmanFilter()
	{
		initialize(cv::Rect());
		m_lastUpdate = 0;
		m_lastPredict = 0;
	}

	void KalmanFilter(cv::Rect initRect)
	{
		initialize(initRect);
		m_lastUpdate = 0;
		m_lastPredict = 0;
		m_counter++;
	}

	cv::Rect predict()
	{
		cv::Mat p = m_kf.predict();
		std::cout << p << std::endl;

		if (m_lastUpdate > 0)
			m_lastPredict = 0;

		m_lastUpdate++;

		cv::Rect predictBox = invRectTransform(p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0), p.at<double>(3, 0));

		m_history.push_back(predictBox);
		return m_history.back();
	}

	cv::Rect curState()
	{
		cv::Mat s = m_kf.statePost;
		std::cout << s << std::endl;
		return invRectTransform(s.at<double>(0, 0), s.at<double>(1, 0), s.at<double>(2, 0), s.at<double>(3, 0));
	}


	void update(cv::Rect rect)
	{
		m_lastPredict++;
		m_lastUpdate = 0;

		m_history.clear();

		m_meas.at<double>(0, 0) = rect.x + rect.width / 2.;
		m_meas.at<double>(1, 0) = rect.y + rect.height / 2.;
		m_meas.at<double>(2, 0) = rect.area();
		m_meas.at<double>(3, 0) = rect.width / rect.height;

		m_kf.correct(m_meas);
	}

};