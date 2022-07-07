#include "CameraMotionEstimation.h"

#include "opencv2/core.hpp"
#include <opencv2/calib3d.hpp>

	void CameraMotionEstimator::findShiTomasiCorners(cv::Mat first_frame, int maxCorners /*= 50*/, double qualityLevel /*= 0.04*/, double minDistance /*= 5*/, int blockSize /*= 3*/)
	{
		cvtColor(first_frame, m_first, cv::COLOR_BGR2GRAY);
		goodFeaturesToTrack(m_first, m_p0, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, 0.04);
		m_pimg = first_frame.clone();
	}

	void CameraMotionEstimator::evalOpticalFlow(cv::Mat frame)
	{
		// Creating a mask image for drawing purposes
		cv::Mat mask = cv::Mat::zeros(m_pimg.size(), m_pimg.type());

		while (true) {

			cv::Mat gray_frame;
			cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

			// calculating optical flow
			std::vector<uchar> status;
			std::vector<float> err;

			cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
			cv::calcOpticalFlowPyrLK(m_first, gray_frame, m_p0, m_p1, status, err, cv::Size(15, 15), 2, criteria);

			std::vector<cv::Point2f> good_new;
			std::vector<cv::Scalar> colors;
			cv::RNG rng;

			for (int i = 0; i < 100; i++)
			{
				int r = rng.uniform(0, 256);
				int g = rng.uniform(0, 256);
				int b = rng.uniform(0, 256);

				colors.push_back(cv::Scalar(r, g, b));
			}

			for (uint i = 0; i < m_p0.size(); i++)
			{
				// Select good points
				if (status[i] == 1) {
					good_new.push_back(m_p1[i]);

					// draw the tracks
					line(mask, m_p1[i], m_p0[i], colors[i], 2, cv::LINE_AA);
					circle(m_pimg, m_p1[i], 5, colors[i], -1, cv::LINE_AA);
				}
			}

			m_pimg = frame;
			m_first = gray_frame.clone();
			m_p0 = good_new;
			cv::Mat img;

			add(frame, mask, img);
			imshow("Frame", img);

			int keyboard = cv::waitKey(0);
			if (keyboard == 'q' || keyboard == 27)
				break;
		}

	}

	void CameraMotionEstimator::evalHomography(int Min_Match /*= 10*/)
	{
		if (m_p0.size() > Min_Match && m_p0.size() == m_p1.size())
		{
			m_H = findHomography(m_p0, m_p1);
			std::cout << "H:\n" << m_H << std::endl;

			cv::Mat H_inv = m_H.inv();
			perspectiveTransform(m_p1, m_chkp0, H_inv);
			float est, eps = 1e-6;
			std::size_t match = 0;

			for (int i = 0; i < m_p0.size(); i++)
			{
				float est_x = m_p0[i].x - m_chkp0[i].x;
				float est_y = m_p0[i].y - m_chkp0[i].y;
				est = sqrtf(powf(est_x, 2) + powf(est_y, 2));

				if (est < eps) {
					match++;
				}
			}

			if (match / m_p0.size() > 0.9) {
				std::cout << "Homography matrix is correct" << std::endl;
			}
			else std::cout << "Homography matrix is not correct" << std::endl;
		}
	}
