// Test program for CameraMotionEstimator and KalmannFilter classes.
//

#include <iostream>
#include <fstream>
#include <stdexcept>
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/calib3d.hpp>
#include <string>
#include "CameraMotionEstimation.h"
#include "KalmanFilter.h"



std::string read_file(const std::string& file_name)
{
	auto ss = std::ostringstream{};
	std::ifstream csv_file("C:\\Users\\user\\Documents\\Dataset_UAV123_10fps\\UAV123_10fps\\anno\\UAV123_10fps\\bike1.txt");
	if (!csv_file.is_open())
	{
		throw std::runtime_error("could not open the file:");
	}
	ss << csv_file.rdbuf();
	return ss.str();
}

std::vector<cv::Rect> csv_maker_model(const std::string& csv_data)
{
	std::vector<std::string> values;
	std::vector<cv::Rect> outRects;

	std::istringstream sstream(csv_data);
	std::string delimiter = ",";

	std::string record;
	std::getline(sstream, record);
	while (std::getline(sstream, record))
	{
		std::size_t pos = 0;
		while ((pos = record.find(delimiter)) != std::string::npos)
		{
			values.push_back(record.substr(0, pos));
			record.erase(0, pos + delimiter.length());
		}
		if (!record.empty())
			values.push_back(record);
		try
		{
			const int topx = std::stoi(values[0]);
			const int topy = std::stoi(values[1]);
			
			const int width = std::stoi(values[2]);
			const int height = std::stoi(values[3]);
			outRects.push_back(cv::Rect(topx, topy, width, height));
		}
		catch (const std::invalid_argument& badArg)
		{
			throw std::runtime_error("an error occurred while csv parsing" + std::string(badArg.what()));
		}
		catch (const std::out_of_range& outRange)
		{
			throw std::runtime_error("an error occurred while csv parsing" + std::string(outRange.what()));
		}
		catch (...)
		{
			throw std::runtime_error("an undefined error occurred while csv parsing");
		}
		values.clear();
	}
	return outRects;
}




int main(int argc, char **argv)
{
	const std::string file_name = "path";
	const auto file_str = read_file(file_name);
	const auto rects = csv_maker_model(file_str);

	cv::VideoCapture sequence("path\\%06d.jpg");
	if (!sequence.isOpened())
	{
		std::cerr << "Failed to open the image sequence!\n" << std::endl;
		return 1;
	}

	cv::Mat frame;
	sequence >> frame;

	
	//CameraMotionEstimator Estimate; Uncomment for CameraMotionEstimator class test
	//Estimate.Find_Shi_Tomasi_corners(frame);

	KalmannFilter kf;
	kf.KalmanFilter(rects[1]);

	for (int i=0; i < 1000; i++ )
	{
		sequence >> frame;
		if (frame.empty())
		{
			std::cout << "End of Sequence" << std::endl;
			break;
		}

		//Estimate.OpticalFlow(frame);
		//Estimate.Homography();
		cv::Rect PredictBox = kf.predict();
		kf.update(rects[i]);
		cv::Rect StateBox = kf.curState();
		cv::rectangle(frame, rects[i], cv::Scalar(0, 255, 0));
		cv::rectangle(frame, PredictBox, cv::Scalar(0, 0, 255));

		imshow("Image sequence | press ESC to close", frame);
		
		if (cv::waitKey(500) == 27)
			break;
	}
	
	return 0;
}


