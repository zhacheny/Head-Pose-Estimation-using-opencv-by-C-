#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include "head_pose_estimation.h"

using namespace std;
using namespace cv;

sFaceAngles head_pose_estimation::face_orientation(cv::Mat im, int landmarks[])
{
	float Bval = 0.0;
	float Gval = 0.0;
	float Rval = 0.0;
	int i, j;
	int count = 0;
	for (i = 0; i < im.cols; ++i){
		for (j = 0; j < im.rows; ++j){
			Bval += im.at<cv::Vec3b>(j, i)[0];
			Gval += im.at<cv::Vec3b>(j, i)[1];
			Rval += im.at<cv::Vec3b>(j, i)[2];
			count += 1;
		}
	}
	Bval /= count;
	Gval /= count;
	Rval /= count;
	cout << Bval<<endl<< Gval<<endl<<Rval<<endl;
	if (Bval > 50 && Gval > 5 && Rval > 50){
		float mx = (landmarks[6] + landmarks[8]) / 2;
		float my = (landmarks[7] + landmarks[9]) / 2;
		//2D image points. If you change the image, you need to change vector
		std::vector<cv::Point2d> image_points;
		image_points.push_back(cv::Point2d(landmarks[4], landmarks[5]));    // Nose tip
		//image_points.push_back(cv::Point2d(169, 215));    // Chin
		image_points.push_back(cv::Point2d(landmarks[0], landmarks[1]));     // Left eye left corner
		image_points.push_back(cv::Point2d(landmarks[2], landmarks[3]));    // Right eye right corner
		//image_points.push_back(cv::Point2d(landmarks[6], landmarks[7]));    // Left Mouth corner
		//image_points.push_back(cv::Point2d(landmarks[8], landmarks[9]));    // Right mouth corner
		image_points.push_back(cv::Point2d(mx, my));		//mouth center

		// 3D model points.
		std::vector<cv::Point3d> model_points;
		model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
		model_points.push_back(cv::Point3d(-165.0f, 170.0f, -135.0f));       // Left eye left corner
		model_points.push_back(cv::Point3d(165.0f, 170.0f, -135.0f));        // Right eye right corner
		//model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));		// left mouth corner
		//model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));		// right mouth corner
		model_points.push_back(cv::Point3d(0.0f, -150.0f, -125.0f));		// mouth center

		// Camera internals
		double focal_length = im.cols / 2 / tan((60 / 2) * (PI / 180)); // Approximate focal length.

		Point2d center = cv::Point2d(im.cols / 2, im.rows / 2);
		cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);

		cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

		//cout << "Camera Matrix " << endl << camera_matrix << endl;
		// Output rotation and translation
		cv::Mat rotation_vector; // Rotation in axis-angle form
		cv::Mat translation_vector;
		cv::Mat rvec_matrix;
		cv::Mat proj_matrix;
		cv::Mat eulerAngles;
		cv::Mat cameraMatrix;
		cv::Mat rotMatrix;
		cv::Mat transVect;
		double pitch;
		double yaw;
		double roll;
		// Solve for pose
		cv::solvePnP(model_points, image_points, camera_matrix,
			dist_coeffs, rotation_vector, translation_vector, false, SOLVEPNP_UPNP);


		//cout << nose_end_point2D << endl;
		Rodrigues(rotation_vector, rvec_matrix, noArray());
		//cout << "Rodrigues Vector" << endl << rvec_matrix << endl;

		hconcat(rvec_matrix, translation_vector, proj_matrix);
		//cout << "proj_matrix Vector" << endl << proj_matrix << endl;

		decomposeProjectionMatrix(proj_matrix, cameraMatrix, rotMatrix, transVect, noArray(),
			noArray(), noArray(), eulerAngles);

		//cout << "eulerAngles" << endl << eulerAngles << endl;
		pitch = degreesToRadians(eulerAngles.at<double>(0));
		yaw = degreesToRadians(eulerAngles.at<double>(1));
		roll = degreesToRadians(eulerAngles.at<double>(2));
		pitch = RadiansTodegrees(asin(sin(pitch)));
		yaw = RadiansTodegrees(asin(sin(yaw)));
		roll = RadiansTodegrees(asin(sin(roll)));

		sFaceAngles l;
		l.pitch = pitch;
		l.roll = roll;
		l.yaw = yaw;
		return l;
	}
	else{
		cout << "Unidentificable";
	}
}
