#include "wrist.h"

void wrist::wristRecog(Mat colorImg,Point2f& center,double& angle)
{
	Mat grayImg,binary;
	cvtColor(colorImg, grayImg, COLOR_BGR2GRAY);
	threshold(grayImg, binary, 80, 255, THRESH_BINARY);
	imshow("binary", binary);
	waitKey(0);

	vector<vector<Point>> contours;
	findContours(binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	if (!contours.size())
	{
		cout << "contours not exits" << endl;
		return;
	}

	vector<RotatedRect> boundRect(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		//drawContours(copyimg, contours, i, Scalar(0, 0, 255), 1);
		boundRect[i] = minAreaRect(Mat(contours[i]));
		if (contours[i].size() > MinArea && contours[i].size() < MaxArea)
		{
			drawContours(colorImg, contours, i, Scalar(0, 0, 255), 3);
			
			Moments M = moments(contours[i]);
			center.x = M.m10 / M.m00;
			center.y = M.m01 / M.m00;
			circle(colorImg, center, 3, Scalar(0, 0, 255), -1);
			cout << "pixel:(" << center.x << "," << center.y << ")" << "     ";
			//cout << contours[i].size() << endl;

			if (boundRect[i].size.height > boundRect[i].size.width)
				angle = 90.0 + abs(boundRect[i].angle);
			else angle = abs(boundRect[i].angle);
			//cout << angle << endl;
		}

	}

	// imshow("result", colorImg);
	// waitKey(0);

	
	int x_offset = 160;
	int y_offset = 100;

	vector<double>p(3);//012--xyz
	
	p[2] = 150 / camera_factor_wrist;
	p[0] = (center.x + double(x_offset) - camera_cx_wrist) * p[2] / camera_fx_wrist;
	p[1] = (center.y + double(y_offset) - camera_cy_wrist) * p[2] / camera_fy_wrist;
	 
	center.x = p[0];
	center.y = p[1];

}

