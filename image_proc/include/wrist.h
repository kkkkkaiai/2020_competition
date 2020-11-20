#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <string.h>

using namespace std;
using namespace cv;

class wrist
{
public:
	int MinArea = 200, MaxArea = 600;
	void wristRecog(Mat colorImg, Point2f& center, double& angle);

};
