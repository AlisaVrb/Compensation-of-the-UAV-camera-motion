# Stabilization module
### The purpose of this work is to develop and implement an algorithm for compensating the UAV camera's own motion in the task of tracking an object in a video stream

At the moment, the repository contains two wrapper classes for the OpenCV API:
- CameraMotionEstimator class that implements the estimation of camera movement by finding the homography matrix between adjacent frames.
- KalmanFilter class that predicts the position of an object in the next frame.
### In this work were used:
- [UAV123 dataset](https://cemse.kaust.edu.sa/ivul/uav123) and [UAVDT dataset](https://sites.google.com/view/grli-uavdt/%E9%A6%96%E9%A1%B5) for testing the CameraMotionEstimator class (Image sequence: %06d.jpg).
- [UAV123 annotation dataset](https://cemse.kaust.edu.sa/ivul/uav123) for testing the  KalmanFilter class.
- OpenCV 4.5.5
