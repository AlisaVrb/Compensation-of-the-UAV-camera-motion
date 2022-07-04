// KCFtrack_opencv.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <cstring>
using namespace std;
using namespace cv;


class CameraMotionEstimator
{
private:
	Mat img_pr; // previous frame
	Mat img_cur; // current frame
	vector<Point2f> p0, p1, p0_check; // p0 - previous points, p1 - current points
	Mat gray_firstframe; 
	Mat H;
	int match;
public:
	void Find_Shi_Tomasi_corners(Mat first_frame, int maxCorners = 50, double qualityLevel = 0.04, double minDistance = 5, int blockSize = 3)
	{
		cvtColor(first_frame, gray_firstframe, COLOR_BGR2GRAY);
		goodFeaturesToTrack(gray_firstframe, p0, maxCorners, qualityLevel, minDistance, Mat(), blockSize, false, 0.04);
		img_pr = first_frame.clone();
	}

	void OpticalFlow(Mat frame)
	{
		// Creating a mask image for drawing purposes
		Mat mask = Mat::zeros(img_pr.size(), img_pr.type());
		while (true){
			Mat gray_frame;
			cvtColor(frame, gray_frame, COLOR_BGR2GRAY);
			// calculating optical flow
			vector<uchar> status;
			vector<float> err;
			TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
			calcOpticalFlowPyrLK(gray_firstframe, gray_frame, p0, p1, status, err, Size(15, 15), 2, criteria);
			vector<Point2f> good_new;
			vector<Scalar> colors;
			RNG rng;
			for (int i = 0; i < 100; i++)
			{
				int r = rng.uniform(0, 256);
				int g = rng.uniform(0, 256);
				int b = rng.uniform(0, 256);
				colors.push_back(Scalar(r, g, b));
			}

			for (uint i = 0; i < p0.size(); i++)
			{
				// Select good points
				if (status[i] == 1) {
					good_new.push_back(p1[i]);
					// draw the tracks
					line(mask, p1[i], p0[i], colors[i], 2, LINE_AA);
					circle(img_pr, p1[i], 5, colors[i], -1, LINE_AA);
				}
			}
			img_pr = frame;
			gray_firstframe = gray_frame.clone();
			p0 = good_new;
			Mat img;
			add(frame, mask, img);
			imshow("Frame", img);
			int keyboard = waitKey(0);
			if (keyboard == 'q' || keyboard == 27)
				break;
		}
		
	}

	void Homography(int Min_Match = 10)
	{
		if (p0.size() > Min_Match && p0.size() == p1.size())
		{
			H = findHomography(p0, p1);
			cout << "H:\n" << H << endl;
			Mat H_inv= H.inv();
			perspectiveTransform(p1, p0_check, H_inv);	
			float est, eps = 0.0000001;
			for (int i = 0; i < p0.size(); i++)
			{
				float est_x = p0[i].x - p0_check[i].x;
				float est_y = p0[i].y - p0_check[i].y;
				est = sqrtf(powf(est_x, 2) + powf(est_y, 2));
				if (est < eps) {
					match++;
				}
			}
			if (match/p0.size() > 0.9) {
				cout << "Homography matrix is correct" << endl;
			}
			else cout << "Homography matrix is incorrect" << endl;
		}
	}
};


	int main(int argc, char **argv)
{
		
			VideoCapture sequence("C:\\Users\\user\\Downloads\\UAV-benchmark-S\\UAV-benchmark-S\\S0101\\img%06d.jpg");
			if (!sequence.isOpened())
			{
				cerr << "Failed to open the image sequence!\n" << endl;
				return 1;
			}
			Mat frame;
			sequence >> frame;
			CameraMotionEstimator Estimate;
			Estimate.Find_Shi_Tomasi_corners(frame);
			for (;;)
			{
				sequence >> frame;
				if (frame.empty())
				{
					cout << "End of Sequence" << endl;
					break;
				}
				Estimate.OpticalFlow(frame);
				Estimate.Homography();
				
				//imshow("Image sequence | press ESC to close", frame);

				/*if (waitKey(500) == 27)*/
					/*break;*/
			}
			
			return 0;
}

