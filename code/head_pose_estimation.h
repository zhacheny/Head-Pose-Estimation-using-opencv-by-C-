#pragma once
#include "stdafx.h"
#include <opencv2/opencv.hpp>

typedef struct {
	double pitch;
	double roll;
	double yaw;
} sFaceAngles;;

class head_pose_estimation
{
public:
	sFaceAngles face_orientation(cv::Mat im, int landmarks[]);

	double degreesToRadians(double angle_in_degrees){
		return angle_in_degrees * (PI / 180.0);
	}
	double RadiansTodegrees(double degrees_in_angle){
		return degrees_in_angle * (180.0 / PI);
	}
	
};

