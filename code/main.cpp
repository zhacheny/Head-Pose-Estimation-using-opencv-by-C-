#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include "head_pose_estimation.h"

using namespace std;
using namespace cv;


int main(int argc, char **argv){
	double pitch = 0;
	double roll = 0;
	double yaw = 0;
	//Read input image
	cv::Mat im = cv::imread("C:\\Users\\dell\\PycharmProjects\\pycaffe-mtcnn\\test\\shot\\shot000000_2018-05-24_21-45-39_54.613.jpg");
	fstream file;
	file.open("C:\\Users\\dell\\PycharmProjects\\pycaffe-mtcnn\\landmark.txt", ios::in);
	//file.open("s.txt", ios::in);
	if (!file)
		cout << "file not founded" << endl;
	int a[100];
	int pos = 0;
	while (!file.eof())
	{
		file >> a[pos];
		pos++;
		if (pos >= 100)
			break;
	}
	file.close();
	/*for (int i = 0; i<pos; i++)
	cout << a[i] << endl;*/
	sFaceAngles output;
	head_pose_estimation fb;
	output = fb.face_orientation(im, a);
	cout << "pitch:" << output.pitch << endl << "roll:" << output.roll << endl << "yaw:" << output.yaw << endl;
	if (abs(output.pitch)>70 || abs(output.roll) > 70 || abs(output.yaw) > 70){
		cout << "side face!";
	}
	else{
		cout << "front face!";
	}
	system("Pause");
}
