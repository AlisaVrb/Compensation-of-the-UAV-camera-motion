#include "CameraMotionEstimation.h"

#include "opencv2/core.hpp"
#include <opencv2/calib3d.hpp>

class CameraMotionEstimator
{
private:

	cv::Mat img_cur; // current frame

	std::vector<cv::Point2f> p0, p1, p0_check; // p0 - previous points, p1 - current points

	cv::Mat gray_firstframe;

	cv::Mat H;

	int match;

public:

	void Find_Shi_Tomasi_corners(cv::Mat first_frame, int maxCorners = 50, double qualityLevel = 0.04, 
		double minDistance = 5, int blockSize = 3)
	{
		cvtColor(first_frame, gray_firstframe, cv::COLOR_BGR2GRAY);
		goodFeaturesToTrack(gray_firstframe, p0, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, 0.04);
		img_pr = first_frame.clone();
	}

	void OpticalFlow(cv::Mat frame)
	{
		// Creating a mask image for drawing purposes
		cv::Mat mask = cv::Mat::zeros(img_pr.size(), img_pr.type());

		while (true) {

			cv::Mat gray_frame;
			cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

			// calculating optical flow
			std::vector<uchar> status;
			std::vector<float> err;

			cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
			cv::calcOpticalFlowPyrLK(gray_firstframe, gray_frame, p0, p1, status, err, cv::Size(15, 15), 2, criteria);

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

			for (uint i = 0; i < p0.size(); i++)
			{
				// Select good points
				if (status[i] == 1) {
					good_new.push_back(p1[i]);

					// draw the tracks
					line(mask, p1[i], p0[i], colors[i], 2, cv::LINE_AA);
					circle(img_pr, p1[i], 5, colors[i], -1, cv::LINE_AA);
				}
			}

			img_pr = frame;
			gray_firstframe = gray_frame.clone();
			p0 = good_new;
			cv::Mat img;

			add(frame, mask, img);
			imshow("Frame", img);

			int keyboard = cv::waitKey(0);
			if (keyboard == 'q' || keyboard == 27)
				break;
		}

	}

	void Homography(int Min_Match = 10)
	{
		if (p0.size() > Min_Match && p0.size() == p1.size())
		{
			H = findHomography(p0, p1);
			std::cout << "H:\n" << H << std::endl;

			cv::Mat H_inv = H.inv();
			perspectiveTransform(p1, p0_check, H_inv);
			float est, eps = 1e-6;
				
			for (int i = 0; i < p0.size(); i++)
			{
				float est_x = p0[i].x - p0_check[i].x;
				float est_y = p0[i].y - p0_check[i].y;
				est = sqrtf(powf(est_x, 2) + powf(est_y, 2));

				if (est < eps) {
					match++;
				}
			}

			if (match / p0.size() > 0.9) {
				std::cout << "Homography matrix is correct" << std::endl;
			}
			else std::cout << "Homography matrix is not correct" << std::endl;
		}
	}
};