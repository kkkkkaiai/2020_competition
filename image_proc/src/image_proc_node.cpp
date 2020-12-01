#include <iostream>
#include <vector>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_proc_msgs/ImageProcSrv.h>
#include <image_proc_msgs/ImageProcMsg.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#define DEBUG_MODE
using namespace std;
using namespace cv;

enum
{
	Processor_cl,
	Processor_gl,
	Processor_cpu
};

// head_camera
const float camera_factor = 1;
const float camera_cx = 960;
const float camera_cy = 540;
const float camera_fx = 1.081e+03;
const float camera_fy = 1.081e+03;

// wrist_camera
const float camera_factor_wrist = 1;
float camera_cx_wrist;
float camera_cy_wrist;
float camera_fx_wrist;
float camera_fy_wrist;

//const int MinArea = 150;
//const int MaxArea = 600;

vector<int>area4wristA{ 150,600 };
vector<int>area4wristB{ 300,700 };

int suppres_min = 955, suppres_max = 1180;
int spprs_xmin = 600, spprs_xmax = 1470, spprs_ymin = 450, spprs_ymax = 880;
int MinArea_vtc = 40, MaxArea_vtc = 160;
int MinArea_clr = 50, MaxArea_clr = 210, MaxArea_overlap = 450;
int MinArea_clr_hori = 100, MaxArea_clr_hori = 350;
int gray_supp = 120;
vector<int> edge_right{430, 820, 100, 450};
vector<int> edge_left{660, 1270, 170, 400};

int iLowH = 0, iHighH = 255, iLowS = 0, iHighS = 255, iLowV = 100, iHighV = 255;
int iLowH_cup = 0, iHighH_cup = 255, iLowS_cup = 0, iHighS_cup = 15, iLowV_cup = 150, iHighV_cup = 255;
int iLowH_hand_cup = 0, iHighH_hand_cup = 255, iLowS_hand_cup = 0, iHighS_hand_cup = 255, iLowV_hand_cup = 50, iHighV_hand_cup = 255;
int iLowH_hand_bottle = 0, iHighH_hand_bottle = 255, iLowS_hand_bottle = 0, iHighS_hand_bottle = 80, iLowV_hand_bottle = 80, iHighV_hand_bottle = 255;
int iLowH_cup_move = 0, iHighH_cup_move = 180, iLowS_cup_move = 0, iHighS_cup_move = 10, iLowV_cup_move = 2, iHighV_cup_move = 255;

float avgX, avgY;
float avgX_res, avgY_res;

float avgX_cup, avgY_cup;
float avgX_hand_cup, avgY_hand_cup;

float avgX_cir, avgY_cir, settled_z;
float avgX_cir_cup, avgY_cir_cup;
float avgX_cir_hand_cup, avgY_cir_hand_cup;

float g_dConArea, g_dConArea_cup;

#define ver_dep_min 40
#define ver_dep_max 80

struct vision
{
	bool issuccess;
	vector<int> grip_method;		 //1--vertical,0--horizontal
	vector<vector<float>> cntrpoint; //xyz and angel of center point
	vector<int> classify;			 //the classification of the object
};

vision vision_set;

//save image cnt
int cnt = 0;

class ImgProc
{
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = NULL;
	libfreenect2::PacketPipeline *pipeline = NULL;
	string serial;
	int depthProcessor;
	libfreenect2::SyncMultiFrameListener *listener = NULL;
	libfreenect2::Registration *registration = NULL;
	libfreenect2::FrameMap frames;
	Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	ros::ServiceServer server_;
	image_transport::Subscriber sub_left_cam;
	image_transport::Subscriber sub_right_cam;
	image_transport::Subscriber image_color_sub_;
	image_transport::Subscriber image_depth_sub_;

	cv_bridge::CvImagePtr left_cv_Img_ptr;
	cv_bridge::CvImagePtr right_cv_Img_ptr;
	cv_bridge::CvImagePtr realsense_color_ptr;
	cv_bridge::CvImagePtr realsense_depth_ptr;

	Mat left_color_image;
	Mat right_color_image;

	Mat realsense_colorImage;
	Mat realsense_depthImage;

public:
	ImgProc(ros::NodeHandle &nh)
		: nh_(nh), it_(nh)
	{
		server_ = nh_.advertiseService("Image_Process_A", &ImgProc::imageProcessCallback, this);
		sub_left_cam = it_.subscribe("/cameras/left_hand_camera/image", 1, &ImgProc::lWristImgProcessCallbk, this);
		sub_right_cam = it_.subscribe("/cameras/right_hand_camera/image", 1, &ImgProc::rWristImgProcessCallbk, this);
		image_color_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImgProc::imageRealsenseColorCb, this);
		image_depth_sub_ = it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &ImgProc::imageRealsenseDepthCb, this);

		realsense_colorImage = Mat::zeros(1280, 720, CV_8UC3);
		realsense_depthImage = Mat::zeros(1280, 720, CV_16UC1);

		if (freenect2.enumerateDevices() == 0)
		{
			std::cout << "no device connected!" << std::endl;
			return;
		}
		serial = freenect2.getDefaultDeviceSerialNumber();
		std::cout << "SERIAL: " << serial << std::endl;
		int depthProcessor = Processor_gl;
		if (depthProcessor == Processor_cpu)
		{
			if (!pipeline)
				//! [pipeline]
				pipeline = new libfreenect2::CpuPacketPipeline();
			cout << "choose cpu mode" << endl;
			//! [pipeline]
		}
		else if (depthProcessor == Processor_gl) // if support gl
		{
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
			if (!pipeline)
			{
				pipeline = new libfreenect2::OpenGLPacketPipeline();
				cout << "choose gl mode" << endl;
			}
#else
			std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
		}
		else if (depthProcessor == Processor_cl) // if support cl
		{
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
			if (!pipeline)
				pipeline = new libfreenect2::OpenCLPacketPipeline();
			cout << "choose cl mode" << endl;
#else
			std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
		}
		if (pipeline)
		{
			dev = freenect2.openDevice(serial, pipeline);
		}
		else
		{
			dev = freenect2.openDevice(serial);
		}
		if (dev == 0)
		{
			std::cout << "failure opening device!" << std::endl;
			return;
		}

		listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir);

		dev->setColorFrameListener(listener);
		dev->setIrAndDepthFrameListener(listener);
		dev->start();

		std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
		std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

		registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

		ROS_INFO("Kinect initial successful");
	}

	~ImgProc()
	{
		dev->stop();
		dev->close();
		delete listener;
		delete pipeline;
		delete registration;
	}

	void imageRealsenseColorCb(const sensor_msgs::ImageConstPtr &msg)
	{
		try
		{
			realsense_color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			realsense_colorImage = realsense_color_ptr->image;
			// ROS_INFO("RGB.CHANNLES=%d", realsense_colorImage.channels());
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("Realsense color exception: %s", e.what());
			return;
		}
	}

	void imageRealsenseDepthCb(const sensor_msgs::ImageConstPtr &msg)
	{
		try
		{
			realsense_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
			realsense_depthImage = realsense_depth_ptr->image;
			// ROS_INFO("RGB.CHANNLES=%d", realsense_depthImage.channels());
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("Realsense color exception: %s", e.what());
			return;
		}
	}

	void lWristImgProcessCallbk(const sensor_msgs::ImageConstPtr &msg)
	{
		try
		{
			left_cv_Img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			left_color_image = left_cv_Img_ptr->image;
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	}

	void rWristImgProcessCallbk(const sensor_msgs::ImageConstPtr &msg)
	{
		try
		{
			right_cv_Img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			right_color_image = right_cv_Img_ptr->image;
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	}

	bool imageProcessCallback(image_proc_msgs::ImageProcSrv::Request &req,
							  image_proc_msgs::ImageProcSrv::Response &resp)
	{
		libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
		listener->waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		// libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		// cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
		cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

		registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

		// cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
		cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
		cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

		listener->release(frames);
		// cv::imshow("img", depthmat);
		// cv::waitKey(1000);
		ROS_INFO("ImgProc service call success");
		ROS_INFO("Request method is: %d", req.method);

		vector<float> result;
		bool vision_set_judge = false;
		bool taskA_fun1_judge = false;

		// try
		// {
		if (req.method == -1)
		{
			depthimg_save(rgbd2, rgbmat);
			return true;
		}
		else
		{
			switch (req.method)
			{
			case 0:
				result = single(rgbd2, rgbmat);
				break;
			case 1:
				result = wristRecog(left_color_image, 1, 215, 80, area4wristA);
				break; // horizontal
			case 2:
				result = wristRecog(right_color_image, 0, 215, 80, area4wristA);
				break;
			case 4:
				result = wristRecog(left_color_image, 1, 190, 80, area4wristA);
				break; // verticle
			case 5:
				result = wristRecog(right_color_image, 0, 190, 80, area4wristA);
				break;
			case 6:
				result = bottle_detect(rgbmat, rgbd2, 1);
				break; // task C // detect cup
			case 7:
				result = bottle_detect(rgbmat, rgbd2, 0);
				break; // detect bottle
			case 8:
				result = recog_on_hand(realsense_colorImage, edge_right, realsense_depthImage, 1);
				break; //right up
			case 9:
				result = hand_cup_detect(left_color_image, 1);
				break;
			case 10:
				result = hand_cup_detect(right_color_image, 0);
				break;
			case 11:
				result = hand_bottle_detect(left_color_image, 1);
				break;
			case 12:
				result = hand_bottle_detect(right_color_image, 0);
				break;
			case 14:
				result = recog_on_hand(realsense_colorImage, edge_left, realsense_depthImage, 0);
				break; //left up
			case 15:
				result = cup_detect(rgbmat, rgbd2);
				break;
			case 16:
				result = overlap(left_color_image, 1, 210);
				break;
			case 17:
				result = overlap(right_color_image, 0, 210);
				break;
			case 18:
				result = wristRecog(left_color_image, 1, 225, 200, area4wristB);
				break;
			// case 19:
			// 	result = wristRecog(right_color_image, 0, 225, 200, area4wristB);
			// 	break;
			default:
				vision_set_judge = true;
			}
			if (vision_set_judge)
			{
				switch (req.method)
				{
				case 3:
					taskA_fun1(rgbmat, rgbd2);
					ROS_INFO("3");
					break;
				case 13:
					taskB(rgbmat, rgbd2);
					ROS_INFO("13");
					break;
				default:
					taskA_fun1_judge = true;
				}
				if (taskA_fun1_judge)
				{
					ROS_ERROR("Request Method ERROR");
					return false;
				}
				else
				{
					resp.is_success = vision_set.issuccess;
					resp.grip_method = vision_set.grip_method;
					for (auto i : vision_set.cntrpoint)
					{
						image_proc_msgs::ImageProcMsg temp;
						vector<float> pos(i.begin(), i.begin() + 3);
						vector<float> orient(i.begin() + 3, i.end());
						temp.position = pos;
						temp.orientation = orient;

						resp.object.push_back(temp);
					}
					resp.classify = vision_set.classify;
				}
			}
			else
			{
				image_proc_msgs::ImageProcMsg temp;
				vector<float> pos(result.begin(), result.begin() + 3);
				vector<float> orient(result.begin() + 3, result.end());
				temp.position = pos;
				temp.orientation = orient;

				resp.object.resize(1);
				resp.object[0] = temp;
				ROS_INFO("ImageProcess handle success!");
			}
		}
		// }
		// catch (ros::Exception e)
		// {
		// 	ROS_ERROR("Image process fail");
		// 	return false;
		// }

		return true;
	}

	//only use to save Image
	bool depthimg_save(const cv::Mat &depimg, cv::Mat colorImg /*, std::string path*/)
	{
		cv::Mat saveimg = cv::Mat(depimg.size(), CV_8UC3);
		for (int i = 0; i < depimg.cols; i++)
		{
			for (int j = 0; j < depimg.rows; j++)
			{
				saveimg.at<Vec3b>(j, i)[0] = int(depimg.at<float>(j, i)) % 256;
				saveimg.at<Vec3b>(j, i)[1] = int(depimg.at<float>(j, i) / 256);
				saveimg.at<Vec3b>(j, i)[2] = 0;
				// saveimg.at<Vec3b>(j, i)[0] = int(depimg.at<unsigned short int>(j, i)) % 256;
				// saveimg.at<Vec3b>(j, i)[1] = int(depimg.at<unsigned short int>(j, i) / 256);
				// saveimg.at<Vec3b>(j, i)[2] = 0;
			}
		}
		cv::imwrite("./" + to_string(cnt) + ".bmp", saveimg);
		cv::imwrite("./" + to_string(cnt) + ".jpg", colorImg);
		cnt++;
		return true;
	}

	//use in code
	bool depthimg_save2(const cv::Mat &depimg, std::string path)
	{
		// cv::Mat saveimg = cv::Mat(depimg.size(), CV_8UC3);
		// for (int i = 0; i < depimg.rows; ++i)
		// {
		// 	const short int* dep = depimg.ptr<short int>(i);
		// 	uchar* save = saveimg.ptr<uchar>(i);
		// 	for (int j = 0; j < depimg.cols; ++j)
		// 	{
		//     	*save++ = static_cast<uchar>((*dep >> 8) & 0x00ff);
		//     	*save++ = static_cast<uchar>(*dep++ & 0x00ff);
		//     	*save++ = 0;
		// 	}
		// }

		cv::Mat saveimg = cv::Mat(depimg.size(), CV_8UC3);
		for (int i = 0; i < depimg.cols; i++)
		{
			for (int j = 0; j < depimg.rows; j++)
			{
				saveimg.at<Vec3b>(j, i)[0] = int(depimg.at<float>(j, i)) % 256;
				saveimg.at<Vec3b>(j, i)[1] = int(depimg.at<float>(j, i) / 256);
				saveimg.at<Vec3b>(j, i)[2] = 0;
			}
		}
		cv::imwrite(path, saveimg);
		return true;
	}

	void singleobject(Mat colorImg, Mat depImg, float &angle, Point2d &center)
	{
		int spprs_xmin = 480, spprs_xmax = 1400, spprs_ymin = 500, spprs_ymax = 900;
		int MinArea_clr = 100, MaxArea_clr = 500;
		int graysupp = 80;
		Mat grayImg;

		Mat copyimg = colorImg.clone();
		Mat binaryimg = Mat::zeros(colorImg.rows, colorImg.cols, CV_8UC1);

		cvtColor(colorImg, grayImg, COLOR_BGRA2GRAY);

		// imwrite("./gray.jpg",grayImg);
		// depthimg_save2(depImg, "./depth.bmp");

		for (int i = spprs_xmin; i < spprs_xmax; i = i + 1)
			for (int j = spprs_ymin; j < spprs_ymax; j = j + 1)
			{
				if (grayImg.at<uchar>(j, i) < graysupp)
				{
				}
				else
					binaryimg.at<uchar>(j, i) = 255;
			}

		// imshow("binary", binaryimg);
		// waitKey(1000);

		vector<vector<Point>> contours;
		findContours(binaryimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		if (!contours.size())
		{
			cout << "contours not exits" << endl;
			return;
		}
		vector<RotatedRect> boundRect(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			boundRect[i] = minAreaRect(Mat(contours[i]));
			if (contours[i].size() > MinArea_clr && contours[i].size() < MaxArea_clr)
			{
				Point2f vertex[4];
				boundRect[i].points(vertex);

				for (int j = 0; j < 4; j++)
				{
					line(copyimg, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255));
					//vertex[]
				}
				drawContours(copyimg, contours, i, Scalar(0, 0, 255), 1);

				Moments M = moments(contours[i]);
				center.x = M.m10 / M.m00;
				center.y = M.m01 / M.m00;
				circle(copyimg, center, 3, Scalar(0, 0, 255), -1);
				// cout << "(" << center.x << "," << center.y << ")" << "     ";
				// cout << contours[i].size() << endl;

				if (boundRect[i].size.height > boundRect[i].size.width)
					angle = 90.0 + abs(boundRect[i].angle);
				else
					angle = abs(boundRect[i].angle);
			}
		}
		// imshow("result", copyimg);
		// waitKey(1000);
	}

	vector<float> single(Mat &depImg, Mat &colorImg)
	{

		float angle = 0;
		Point2d center = Point2d(0, 0);
		singleobject(colorImg, depImg, angle, center);

		float d = depImg.at<float>(int(center.y), int(center.x));

		if (d == 0)
		{
			int temp_x = int(center.x) - 40, temp_y = int(center.y) - 40;
			while (d == 0 && abs(temp_x - int(center.x)) <= 40 && abs(temp_y - int(center.y)) <= 40)
			{
				d = depImg.at<unsigned short int>(temp_y, temp_x);
				temp_x++;
				temp_y++;
			}
		}

		if (d == 0)
		{
			ROS_ERROR("too many blank in depth image!!");
			cout << center.x << "," << center.y << endl;
		}

		vector<float> p(4); //012--xyz

		p[2] = float(d) / camera_factor * 0.001;
		p[0] = -(center.x - camera_cx) * p[2] / camera_fx;
		p[1] = (center.y - camera_cy) * p[2] / camera_fy;
		p[3] = angle * 3.141592653 / 180;

		ROS_INFO("result: %f, %f, %f, %f.", p[0], p[1], p[2], p[3]);
		return p;
	}

	vector<float> wristRecog(Mat colorImg, int flag, float depth,int greysupp,vector<int>area4wrist)
	{
		imwrite("./hand"+to_string(cnt)+".png",colorImg);
		cnt++;
		if (flag == 0)
		{
			camera_cx_wrist = 488.904;
			camera_cy_wrist = 280.927;
			camera_fx_wrist = 405.821;
			camera_fy_wrist = 405.821;
		}
		else if (flag == 1)
		{
			camera_cx_wrist = 467.294;
			camera_cy_wrist = 340.350;
			camera_fx_wrist = 404.940;
			camera_fy_wrist = 404.940;
		}
		Mat grayImg, binary;
		vector<Point2f> ctrpointpx;
		vector<float> angle_set;
		cvtColor(colorImg, grayImg, COLOR_BGR2GRAY);
		threshold(grayImg, binary, greysupp, 255, THRESH_BINARY);
		//threshold(grayImg, binary, 80, 255, THRESH_BINARY);
		// imshow("binary", binary);
		// waitKey(1000);

		vector<vector<Point>> contours;
		findContours(binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		if (!contours.size())
		{
			ROS_ERROR("contours not exits");
		}

		vector<RotatedRect> boundRect(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			//drawContours(copyimg, contours, i, Scalar(0, 0, 255), 1);
			boundRect[i] = minAreaRect(Mat(contours[i]));
			if (contours[i].size() > area4wrist[0]/*MinArea*/ && contours[i].size() < area4wrist[1]/*MaxArea*/)
			{
				drawContours(colorImg, contours, i, Scalar(0, 0, 255), 3);

				Moments M = moments(contours[i]);
				Point2f center;
				center.x = M.m10 / M.m00;
				center.y = M.m01 / M.m00;

				circle(colorImg, center, 3, Scalar(0, 0, 255), -1);
				//cout << "pixel:(" << center.x << "," << center.y << ")" << "     ";
				//cout << contours[i].size() << endl;
				ctrpointpx.push_back(center);

				float angle;
				if (boundRect[i].size.height > boundRect[i].size.width)
					angle = 90.0 + abs(boundRect[i].angle);
				else
					angle = abs(boundRect[i].angle);
				//cout << angle << endl;
				angle_set.push_back(angle);
			}
		}

		// imshow("result", colorImg);
		// waitKey(1000);

		int ll = decidecp(ctrpointpx, camera_cx_wrist, camera_cy_wrist);

		int x_offset = 0; //160;
		int y_offset = 0; //100;

		cout << "pixel:(" << ctrpointpx[ll].x << "," << ctrpointpx[ll].y << ")" << endl;
		circle(colorImg, ctrpointpx[ll], 3, Scalar(0, 0, 255), -1);

		imshow("result", colorImg);
		waitKey(1000);

		vector<float> p(4); //012--xyz

		p[2] = depth / camera_factor;
		p[0] = (ctrpointpx[ll].x + float(x_offset) - camera_cx_wrist) * p[2] / camera_fx_wrist;
		p[1] = (ctrpointpx[ll].y + float(y_offset) - camera_cy_wrist) * p[2] / camera_fy_wrist;
		p[3] = angle_set[ll];

		return p;
	}

	int decidecp(vector<Point2f> &ctrpointpx, float &camera_cx_wrist, float &camera_cy_wrist)
	{
		if (ctrpointpx.size() == 1)
			return 0;

		float cx = camera_cx_wrist, cy = camera_cy_wrist;
		int n = ctrpointpx.size();
		float min_dis = 10000000;
		int min_index = 0;
		for (int i = 0; i < n; i++)
		{
			float temp = pow((ctrpointpx[i].x - cx), 2) + pow((ctrpointpx[i].y - cy), 2);
			if (min_dis > temp)
			{
				min_dis = temp;
				min_index = i;
			}
		}
		return min_index;
	}

	/***********recognition of multiple objects*********/

	bool depthimg_read2(const std::string path, cv::Mat &depimg)
	{
		cv::Mat saveimg = cv::imread(path);
		depimg = cv::Mat(saveimg.size(), CV_16UC1);
		for (int i = 0; i < depimg.rows; ++i)
		{
			const uchar *save = saveimg.ptr<uchar>(i);
			unsigned short int *dep = depimg.ptr<unsigned short int>(i);
			for (int j = 0; j < depimg.cols; ++j)
			{
				//*dep++ = static_cast<unsigned short int>((*save << 8) | (*(save + 1)));
				*dep++ = static_cast<unsigned short int>(*save + *(save + 1) * 255);
				save += 3;
			}
		}
		return true;
	}

	void findvertical(Mat depImg, vector<Point2f> &crosspcnt,Mat& colorImg,int flag)
	{
		Mat binary = Mat::zeros(depImg.rows, depImg.cols, CV_8UC1);
		for (int i = spprs_xmin; i < spprs_xmax; i = i + 1)
			for (int j = spprs_ymin; j < spprs_ymax; j = j + 1)
			{
				if (depImg.at<unsigned short int>(j, i) >= ver_dep_min && depImg.at<unsigned short int>(j, i) <= ver_dep_max)
				{
					binary.at<uchar>(j, i) = 255;
				}
			}

		//cv::imshow("binary", binary);
		//cv::waitKey(0);
		vector<vector<Point>> contours;
		findContours(binary, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		if (!contours.size())
		{
			cout << "contours not exits" << endl;
			return;
		}
		vector<RotatedRect> boundRect(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			boundRect[i] = minAreaRect(Mat(contours[i]));
			if (contours[i].size() > MinArea_vtc && contours[i].size() < MaxArea_vtc)
			{
				Moments M = moments(contours[i]);
				Point2f center;
				center.x = M.m10 / M.m00;
				center.y = M.m01 / M.m00;

				crosspcnt.push_back(center);
			}
			else if (contours[i].size() > MaxArea_vtc && flag == 0)
			{
				vector<RotatedRect> boundRect(contours.size());
				boundRect[i] = minAreaRect(Mat(contours[i]));
				Point2f vertex[4];
				boundRect[i].points(vertex);
				Point root_points[1][4];
				root_points[0][0] = vertex[0];
				root_points[0][1] = vertex[1];
				root_points[0][2] = vertex[2];
				root_points[0][3] = vertex[3];

				const Point *ppt[1] = {root_points[0]};
				int npt[] = {4};

				polylines(colorImg, ppt, npt, 1, 1, Scalar(0,0,0), 1, 8, 0);
				fillPoly(colorImg, ppt, npt, 1, Scalar(0,0,0));
			}
		}
		// cv::imshow("result", binary);
		// cv::waitKey(1000);
	}

	void caldiff(cv::Mat depImg1, cv::Mat depImg2, cv::Mat &dst)
	{

		dst = cv::Mat::zeros(depImg1.rows, depImg1.cols, CV_16UC1);
		int c = 0;
		for (int i = spprs_xmin; i < spprs_xmax; i = i + 1)
			for (int j = spprs_ymin; j < spprs_ymax; j = j + 1)
			{

				if (depImg1.at<unsigned short int>(j, i) > 0 && depImg2.at<unsigned short int>(j, i) > 0)
				{
					c += abs(depImg1.at<unsigned short int>(j, i) - depImg2.at<unsigned short int>(j, i));
					dst.at<unsigned short int>(j, i) = abs(depImg1.at<unsigned short int>(j, i) - depImg2.at<unsigned short int>(j, i));
				}
			}
	}

	int predict(string path, string savepath)
	{
		Ptr<ml::SVM> svm = ml::SVM::load(savepath);
		Mat image = imread(path);
		Mat img = image.reshape(1, 1);
		img.convertTo(img, CV_32FC1);
		//cout << int(svm->predict(img)) << endl;
		return int(svm->predict(img));
	}

	void datapadding(Mat &roi, int pixmin, int n)
	{
		if (roi.type() != CV_8UC1)
		{
			cout << "input error!" << endl;
			return;
		}
		//int n = 70; //the size of the output image is n * n
		//int pixmin = 50;//if the pixel's value > pixmin, pixel's value = 0;
		Mat padding_roi = Mat::zeros(n, n, CV_8UC1);
		int padd_col = 0, padd_row = 0;
		if (roi.cols < n)
		{
			padd_col = (n - roi.cols) / 2;
		}
		if (roi.rows < n)
		{
			padd_row = (n - roi.rows) / 2;
		}
		for (int i = 0; i < roi.cols; i++)
			for (int j = 0; j < roi.rows; j++)
			{
				if (j + padd_row < n && i + padd_col < n)
				{
					if (roi.at<uchar>(j, i) < pixmin)
					{
						padding_roi.at<uchar>(j + padd_row, i + padd_col) = 0;
					}
					else
					{
						padding_roi.at<uchar>(j + padd_row, i + padd_col) = roi.at<uchar>(j, i);
					}
				}
			}

		roi = padding_roi.clone();
	}

	void rotate_gjx(Mat image, float angle, Point2f center, Point2f vertex[], std::string filename, int pixmin, int n)
	{

		Mat mask = Mat::zeros(image.rows, image.cols, CV_8UC1);
		for (int j = 0; j < 4; j++)
		{
			line(mask, vertex[j], vertex[(j + 1) % 4], Scalar(255));
		}

		Mat rotatemat;
		rotatemat = getRotationMatrix2D(center, angle, 1.0);
		Mat mask_rot, img_rot;

		warpAffine(mask, mask_rot, rotatemat, Size(image.cols, image.rows));
		warpAffine(image, img_rot, rotatemat, Size(image.cols, image.rows));
		vector<vector<Point>> contours;
		findContours(mask_rot, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		if (!contours.size())
		{
			cout << "contours not exits" << endl;
			return;
		}
		vector<Rect> boundRect(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			boundRect[i] = boundingRect(Mat(contours[i]));
			if (contours[i].size() > 40)
			//if (contours[i].size() > MinArea_clr && contours[i].size() < MaxArea_clr)
			{

				Moments M = moments(contours[i]);
				Point2f cenMsk;
				cenMsk.x = M.m10 / M.m00;
				cenMsk.y = M.m01 / M.m00;
				Mat roi = Mat(img_rot, boundRect[i]);
				Mat temp;
				cvtColor(roi, temp, COLOR_BGR2GRAY);
				datapadding(temp, pixmin, n);
				imwrite(filename, temp);
			}
		}
	}

	void findver_color(Mat &colorImg, vector<Point2f> crosspcnt, Mat depImg)
	{
		Mat copyimg = colorImg.clone();
		Mat binaryimg = Mat::zeros(colorImg.rows, colorImg.cols, CV_8UC1);

		Mat grayImg;
		cvtColor(colorImg, grayImg, COLOR_BGR2GRAY);

		for (int i = spprs_xmin; i < spprs_xmax; i = i + 1)
			for (int j = spprs_ymin; j < spprs_ymax; j = j + 1)
			{
				if (grayImg.at<uchar>(j, i) <= gray_supp)
				{
				}
				else
					binaryimg.at<uchar>(j, i) = 255;
			}

		cv::imshow("vertical", binaryimg);
		cv::waitKey(2000);

		vector<vector<Point>> contours;
		findContours(binaryimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		if (!contours.size())
		{
			cout << "contours not exits" << endl;
			return;
		}
		else
		{
			vector<RotatedRect> boundRect(contours.size());
			for (int i = 0; i < contours.size(); i++)
			{
				boundRect[i] = minAreaRect(Mat(contours[i]));
				if (contours[i].size() > MinArea_clr && contours[i].size() < MaxArea_clr && float(boundRect[i].size.height) / float(boundRect[i].size.width) > 0.33 &&
					float(boundRect[i].size.height) / float(boundRect[i].size.width) < 3)
				{
					Moments M = moments(contours[i]);
					Point2f center;
					center.x = M.m10 / M.m00;
					center.y = M.m01 / M.m00;

					for (int k = 0; k < crosspcnt.size(); k++)
					{
						if (abs(crosspcnt[k].x - center.x) < 30 && abs(crosspcnt[k].y - center.y) < 30 && cv::pointPolygonTest(contours[i], crosspcnt[k], false) == 1)
						{
							vision_set.issuccess = 1;
							vision_set.grip_method.push_back(1);

							vector<float> temp;
							temp.push_back(center.x);
							temp.push_back(center.y);
							temp.push_back(1000); //z will be changed in main fun

							float angle;
							if (boundRect[i].size.height > boundRect[i].size.width)
								angle = 90.0 + abs(boundRect[i].angle);
							else
								angle = abs(boundRect[i].angle);
							//cout << "angle:" << angle << endl;
							temp.push_back(angle);

							vision_set.cntrpoint.push_back(temp);

							Point2f vertex[4];
							boundRect[i].points(vertex);
							std::string filename;
							filename = "./o.jpg";
							rotate_gjx(colorImg, -angle, center, vertex, filename, 50, 70);

							int pre_class = predict(filename, "/home/ljq/ros_ws/src/image_proc/param/svm_vertical.xml");
							//cout << "class:" << pre_class << endl;

							vision_set.classify.push_back(pre_class);

							//visualization
							for (int j = 0; j < 4; j++)
							{
								line(copyimg, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255));
							}
							drawContours(copyimg, contours, i, Scalar(0, 0, 255), 1);
							circle(copyimg, center, 3, Scalar(0, 0, 255), -1);
							//cout << "(" << center.x << "," << center.y << ")"<< "     ";
							//cout << contours[i].size() << endl;

							/***fill contours with black***/
							Point root_points[1][4];
							root_points[0][0] = vertex[0];
							root_points[0][1] = vertex[1];
							root_points[0][2] = vertex[2];
							root_points[0][3] = vertex[3];

							const Point *ppt[1] = {root_points[0]};
							int npt[] = {4};

							polylines(colorImg, ppt, npt, 1, 1, Scalar(0, 0, 0), 1, 8, 0);
							fillPoly(colorImg, ppt, npt, 1, Scalar(0, 0, 0));
						}
					}
				}
			}
			cv::imshow("result", copyimg);
			//cv::imshow("colorImg", colorImg);
			cv::waitKey(2000);
		}
	}

#define hor_dep_min 5
#define hor_dep_max 35

	void findhorizontal(Mat colorImg, Mat depImg)
	{

		Mat copyimg = colorImg.clone();
		Mat binaryimg = Mat::zeros(colorImg.rows, colorImg.cols, CV_8UC1);

		Mat grayImg;
		cvtColor(colorImg, grayImg, COLOR_BGR2GRAY);
		GaussianBlur(grayImg, grayImg, Size(5, 5), 0);
		for (int i = spprs_xmin; i < spprs_xmax; i = i + 1)
			for (int j = spprs_ymin; j < spprs_ymax; j = j + 1)
			{
				if (grayImg.at<uchar>(j, i) < gray_supp)
				{
				}
				else if (true /*depImg.at<unsigned short int>(j, i) > hor_dep_min && depImg.at<unsigned short int>(j, i) < hor_dep_max*/)
					binaryimg.at<uchar>(j, i) = 255;
			}

		cv::imshow("horizontal_bi", binaryimg);
		cv::waitKey(2000);

		vector<vector<Point>> contours;
		findContours(binaryimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		if (!contours.size())
		{
			cout << "contours not exits" << endl;
			return;
		}
		else
		{
			ROS_INFO("1111");
			vector<RotatedRect> boundRect(contours.size());
			for (int i = 0; i < contours.size(); i++)
			{
				boundRect[i] = minAreaRect(Mat(contours[i]));
				if (contours[i].size() > MinArea_clr && contours[i].size() <  MaxArea_overlap/*MaxArea_clr*/
					&& float(boundRect[i].size.height) / float(boundRect[i].size.width) > 0.33 &&
					float(boundRect[i].size.height) / float(boundRect[i].size.width) < 3)
				{

					Point2f vertex[4];
					boundRect[i].points(vertex);

					for (int j = 0; j < 4; j++)
					{
						line(copyimg, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255));
					}
					drawContours(copyimg, contours, i, Scalar(0, 0, 255), 1);
					Moments M = moments(contours[i]);
					Point2f center;
					center.x = M.m10 / M.m00;
					center.y = M.m01 / M.m00;
					circle(copyimg, center, 3, Scalar(0, 0, 255), -1);
					//cout << "(" << center.x << "," << center.y << ")" << "     ";
					//cout << "height of the object:" << depImg.at<unsigned short int>(int(center.y), int(center.x)) << endl;
					//cout << "contours size:" << contours[i].size() << endl;

					float angle;
					if (boundRect[i].size.height > boundRect[i].size.width)
						angle = 90.0 + abs(boundRect[i].angle);
					else
						angle = abs(boundRect[i].angle);

					//cout << "angle:" << angle << endl;
					//cout << endl;

					if (center.x > 0 && center.y > 0)
					{
						if (contours[i].size() < MaxArea_clr)
						{
							vision_set.issuccess = 1;
							vision_set.grip_method.push_back(0);
							vector<float>temp;
							temp.push_back(center.x);
							temp.push_back(center.y);
							temp.push_back(1000);
							temp.push_back(angle);
							vision_set.cntrpoint.push_back(temp);
							cout << "center.x=" << center.x << "," << "center.y=" << center.y << endl;
							int objheight = depImg.at<unsigned short int>(int(center.y), int(center.x));
							if (objheight < 15)
							{
								vision_set.classify.push_back(2);
							}
							else
							{
								vision_set.classify.push_back(-1);
							}

						}
						else
						{
							int k = 2;//if overlap, push two same items
							while (k--)
							{
								vision_set.issuccess = 1;
								vision_set.grip_method.push_back(0);
								vector<float>temp;
								temp.push_back(center.x);
								temp.push_back(center.y);
								temp.push_back(1000);
								temp.push_back(angle);
								vision_set.cntrpoint.push_back(temp);
								cout << "center.x=" << center.x << "," << "center.y=" << center.y << endl;
								int objheight = depImg.at<unsigned short int>(int(center.y), int(center.x));
								vision_set.classify.push_back(-2);
							}
						}

					}
				}
			}
			cv::imshow("horizontal", copyimg);
			cv::waitKey(2000);
		}
	}

	void find_stand_obj(Mat colorImg, Mat depobj,int flag)
	{
		vector<Point2f> crosspcnt;
		findvertical(depobj, crosspcnt,colorImg,flag);
		findver_color(colorImg, crosspcnt, depobj);
		if (vision_set.cntrpoint.size() == 0)
		{
			cout << "no vertical objects" << endl;
		}
		else
		{
			cout << "detecte vertical object!" << endl;
		}
	}

	//main function of recognization of multiple objects
	void taskA_fun1(Mat colorImg, Mat depImg)
	{
		//clear the struct vision
		vision_set.issuccess = 0;
		vision_set.classify.clear();
		vision_set.cntrpoint.clear();
		vision_set.grip_method.clear();

		Mat depobj, depImgBG;
		depthimg_save2(depImg, "guojiaxin.bmp");
		imwrite("guojiaxin.jpg", colorImg);

		depthimg_read2("guojiaxin.bmp", depImg);

		depthimg_read2("/home/ljq/ros_ws/src/image_proc/param/0_align.bmp", depImgBG);
		caldiff(depImg, depImgBG, depobj);
		int flag=0;//taskA dont need to filter yuanpan
		find_stand_obj(colorImg, depobj,flag);
		findhorizontal(colorImg, depobj);
		cout << "the size of vision_set is:" << vision_set.cntrpoint.size() << endl;
		//cout << "the size of  is:" << vision_set.cntrpoint.size() << endl;
		if (vision_set.cntrpoint.size() == 0)
		{
			cout << "no objects!" << endl;
			vision_set.issuccess = 0;
		}
		for (int i = 0; i < vision_set.cntrpoint.size(); i++)
		{

			ROS_INFO("No.%d:  XY-pixel: %f, %f  Class: %d  Grip_Method: %d", i,
					 vision_set.cntrpoint[i][0], vision_set.cntrpoint[i][1],
					 vision_set.classify[i], vision_set.grip_method[i]);
			vision_set.cntrpoint[i][2] = depImg.at<unsigned short int>(int(vision_set.cntrpoint[i][1]), int(vision_set.cntrpoint[i][0])) * 0.001;
			vision_set.cntrpoint[i][0] = -(vision_set.cntrpoint[i][0] - camera_cx) * vision_set.cntrpoint[i][2] / camera_fx;
			vision_set.cntrpoint[i][1] = (vision_set.cntrpoint[i][1] - camera_cy) * vision_set.cntrpoint[i][2] / camera_fy;

			// cout << "No." << i << endl;
			// cout << "xyz-theta:"
			// 	 << "    ";
			// for (int j = 0; j < vision_set.cntrpoint[i].size(); j++)
			// {
			// 	cout << vision_set.cntrpoint[i][j] << "  ";
			// }
			// cout << endl;
			// cout << "class:" << vision_set.classify[i] << endl;
			// cout << "grip method:" << vision_set.grip_method[i] << endl;
			ROS_INFO("No.%d:  XYZ-theta: %f, %f, %f, %f  Class: %d  Grip_Method: %d", i,
					 vision_set.cntrpoint[i][0], vision_set.cntrpoint[i][1], vision_set.cntrpoint[i][2], vision_set.cntrpoint[i][3],
					 vision_set.classify[i], vision_set.grip_method[i]);
		}
	}

	/*recognition  by operating hands*/
	vector<float> hand_process(Mat colorImg, vector<int> area, int padd_length, string pre_path)
	{
		// imshow("after_cut_realsense_img",colorImg);
		// waitKey(1000);
		vector<float> result;
		result.resize(4);
		Mat copyimg = colorImg.clone();
		Mat binaryimg = Mat::zeros(colorImg.rows, colorImg.cols, CV_8UC1);

		Mat grayImg;
		cvtColor(colorImg, grayImg, COLOR_BGR2GRAY);
		GaussianBlur(grayImg, grayImg, Size(5, 5), 0);
		for (int i = 0; i < colorImg.cols; i = i + 1)
			for (int j = 0; j < colorImg.rows; j = j + 1)
			{
				if (grayImg.at<uchar>(j, i) > 200)
				{
					binaryimg.at<uchar>(j, i) = 255;
				}
			}

		cv::imshow("binary", binaryimg);
		cv::waitKey(1000);

		vector<vector<Point>> contours;
		findContours(binaryimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		if (!contours.size())
		{
			cout << "contours not exits" << endl;
			return result;
		}
		else
		{
			vector<RotatedRect> boundRect(contours.size());
			vector<int>contoursizes;
			vector<int>classify;

			for (int i = 0; i < contours.size(); i++)
			{
				boundRect[i] = minAreaRect(Mat(contours[i]));
				ROS_INFO("11111");
				if (contours[i].size() > area[0] && contours[i].size() < area[1] &&
					float(boundRect[i].size.height) / float(boundRect[i].size.width) > 0.33 &&
					float(boundRect[i].size.height) / float(boundRect[i].size.width) < 3)
				{
					ROS_INFO("22222");
					//cout << "ratio:" << float(boundRect[i].size.height) / float(boundRect[i].size.width) << endl;
					Point2f vertex[4];
					boundRect[i].points(vertex);
					for (int j = 0; j < 4; j++)
					{
						line(copyimg, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255));
					}
					drawContours(copyimg, contours, i, Scalar(0, 0, 255), 1);
					//cv::imshow("drawContours", copyimg);
					//cv::waitKey(0);
					Moments M = moments(contours[i]);
					Point2f center;
					center.x = M.m10 / M.m00;
					center.y = M.m01 / M.m00;
					circle(copyimg, center, 3, Scalar(0, 0, 255), -1);

					cout << "(" << center.x << "," << center.y << ")" << "     ";
					cout << "contours size:" << contours[i].size() << endl;

					double angle;
					if (boundRect[i].size.height > boundRect[i].size.width)
						angle = 90.0 + abs(boundRect[i].angle);
					else
						angle = abs(boundRect[i].angle);

					cout << "angle:" << angle << endl;
					cout << endl;
					std::string filename;
					filename = "./p"+to_string(cnt)+".jpg";
					cnt++;
					rotate_gjx(colorImg, -angle, center, vertex, filename, 120, padd_length);
					//cout << pre_path << endl;
					int pre_class = predict(filename, pre_path);
					cout << "class=" << pre_class << endl;
					if (pre_class != 4)
					{
						//result[0] = pre_class;
						contoursizes.push_back(contours[i].size());
						classify.push_back(pre_class);
					}
				}
			}

			if (contoursizes.size() > 1)
			{
				int maxvalue = 0;
				int max_i = 0;
				for (int i = 0; i < contoursizes.size(); i++)
				{
					if (contoursizes[i] > maxvalue)
					{
						maxvalue = contoursizes[i];
						max_i = i;
					}
				}
				result[0] = classify[max_i];
			}
			else if (contoursizes.size() == 0)
			{

			}
			else
			{
				result[0] = classify[0];
			}
			cout<<"final class="<<result[0]<<endl;
			cv::imshow("result", copyimg);
			cv::waitKey(1000);
		}
		return result;
	}
	vector<float> recog_on_hand(Mat colorImg, vector<int> edge, Mat depImg, int method)
	{
		// imshow("realsense_img",colorImg);
		// waitKey(1000);
		depthimg_save(depImg, colorImg);
		for (int i = 0; i < depImg.cols; i = i + 100)
		{
			for (int j = 0; j < depImg.rows; j = j + 100)
			{
				cout << depImg.at<unsigned short int>(j, i) << "   ";
			}
		}
		vector<float> result;
		result.resize(4);

		vector<int> dis;
		vector<int> area2bin; //找区域时候的二值化的面积限制
		vector<int> area;	  //物体（工件or底座）的面积限制
		string pre_path;
		int padd_length;
		//0--left 1--right  2--bottom
		if (method == 0)
		{
			dis = {400, 660};
			area2bin = {1000, 5000};
			//area = { 120 ,500 };//right
			area = {120, 500}; //left
			padd_length = 120;
			pre_path = "/home/ljq/ros_ws/src/image_proc/param/svm_handleft.xml";
		}
		if (method == 1)
		{
			dis = {400, 660};
			area2bin = {1000, 5000};
			//area = { 120 ,500 };//right
			area = {120, 500}; //left
			padd_length = 120;
			pre_path = "/home/ljq/ros_ws/src/image_proc/param/svm_handright.xml";
		}
		//is bottom
		else if (method == 2)
		{
			dis = {350, 660};
			area2bin = {1000, 5000};
			area = {400, 800};
			padd_length = 200;
		}
		Mat copyimg = colorImg.clone();
		Mat binaryimg = Mat::zeros(colorImg.rows, colorImg.cols, CV_8UC1);
		cout << "colorImg.size()" << colorImg.cols << "*" << colorImg.rows << endl;
		Mat grayImg;
		cvtColor(colorImg, grayImg, COLOR_BGR2GRAY);
		int x_min = edge[0], x_max = edge[1], y_min = edge[2], y_max = edge[3];
		for (int i = x_min; i < x_max; i = i + 1)
			for (int j = y_min; j < y_max; j = j + 1)
			{
				if (depImg.at<unsigned short int>(j, i) > dis[0] && depImg.at<unsigned short int>(j, i) < dis[1])
				{
					binaryimg.at<uchar>(j, i) = 255;
				}
			}

		//imshow("binary", binaryimg);
		//waitKey(0);
		vector<vector<Point>> contours;
		findContours(binaryimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		if (!contours.size())
		{

			cout << "contours not exits" << endl;
			return result;
		}
		else
		{
			vector<Rect> boundRect(contours.size());
			for (int i = 0; i < contours.size(); i++)
			{
				if (contours[i].size() > area2bin[0] && contours[i].size() < area2bin[1])
				{
					cout << contours[i].size() << endl;
					boundRect[i] = boundingRect(Mat(contours[i]));
					Mat dst;
					dst = colorImg(boundRect[i]);
					result = hand_process(dst, area, padd_length, pre_path);
				}
			}

			//cv::imshow("result", copyimg);
			//cv::waitKey(1000);
		}
		return result;
	}

	void taskB(Mat colorImg, Mat depImg)
	{
		depthimg_save(depImg,colorImg);
		//clear the struct vision
		vision_set.issuccess = 0;
		vision_set.classify.clear();
		vision_set.cntrpoint.clear();
		vision_set.grip_method.clear();

		Mat depobj, depImgBG;
		depthimg_save2(depImg, "guojiaxin.bmp");
		imwrite("guojiaxin.jpg", colorImg);

		depthimg_read2("guojiaxin.bmp", depImg);

		depthimg_read2("/home/ljq/ros_ws/src/image_proc/param/0_align.bmp", depImgBG);
		caldiff(depImg, depImgBG, depobj);
		find_stand_obj(colorImg, depobj,1);
		Mat copyimg = colorImg.clone();
		Mat binaryimg = Mat::zeros(colorImg.rows, colorImg.cols, CV_8UC1);

		Mat grayImg;
		cvtColor(colorImg, grayImg, COLOR_BGR2GRAY);
		GaussianBlur(grayImg, grayImg, Size(5, 5), 0);
		for (int i = spprs_xmin; i < spprs_xmax; i = i + 1)
			for (int j = spprs_ymin; j < spprs_ymax; j = j + 1)
			{
				if (grayImg.at<uchar>(j, i) < gray_supp)
				{
				}
				else if (true /*depImg.at<unsigned short int>(j, i) > hor_dep_min && depImg.at<unsigned short int>(j, i) < hor_dep_max*/)
					binaryimg.at<uchar>(j, i) = 255;
			}

		vector<vector<Point>> contours;
		findContours(binaryimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		if (!contours.size())
		{

			cout << "contours not exits" << endl;
			return;
		}
		else
		{
			vector<RotatedRect> boundRect(contours.size());
			for (int i = 0; i < contours.size(); i++)
			{
				boundRect[i] = minAreaRect(Mat(contours[i]));
				if (contours[i].size() > 90 && contours[i].size() < 500 &&
					boundRect[i].size.height / boundRect[i].size.width > 1 / 3 && boundRect[i].size.height / boundRect[i].size.width < 3)
				{
					Moments M = moments(contours[i]);
					Point2f center;
					center.x = M.m10 / M.m00;
					center.y = M.m01 / M.m00;
					// cout << "contour.size=" << contours[i].size() << endl;

					float angle;
					if (boundRect[i].size.height > boundRect[i].size.width)
						angle = 90.0 + abs(boundRect[i].angle);
					else
						angle = abs(boundRect[i].angle);
					// cout << "angle:" << angle << endl;

					Point2f vertex[4];
					boundRect[i].points(vertex);
					if (contours[i].size() > 175)
					{
						std::string filename;
						filename = "./o.jpg";
						rotate_gjx(colorImg, -angle, center, vertex, filename, 80, 100);

						int pre_class = predict(filename, "/home/ljq/ros_ws/src/image_proc/param/svm_vbttm.xml");
						cout << "class:" << pre_class << endl;

						if (pre_class != 9)
						{
							vector<float> temp;
							temp.push_back(center.x);
							temp.push_back(center.y);
							temp.push_back(1000);
							temp.push_back(angle);
							vision_set.cntrpoint.push_back(temp);
							vision_set.classify.push_back(pre_class);
							vision_set.grip_method.push_back(2);
							vision_set.issuccess = 1;

							/***fill contours with black***/
							Point root_points[1][4];
							root_points[0][0] = vertex[0];
							root_points[0][1] = vertex[1];
							root_points[0][2] = vertex[2];
							root_points[0][3] = vertex[3];

							const Point *ppt[1] = {root_points[0]};
							int npt[] = {4};

							polylines(colorImg, ppt, npt, 1, 1, Scalar(0, 0, 0), 1, 8, 0);
							fillPoly(colorImg, ppt, npt, 1, Scalar(0, 0, 0));
						}
					}
					//visualization
					for (int j = 0; j < 4; j++)
					{
						line(copyimg, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255));
					}
					drawContours(copyimg, contours, i, Scalar(0, 0, 255), 1);
					circle(copyimg, center, 3, Scalar(0, 0, 255), -1);
					//cout << "(" << center.x << "," << center.y << ")" << ;
					//cout << contours[i].size() << endl;
				}
			}
			cv::imshow("result", copyimg);
			cv::waitKey(1000);
		}

		findhorizontal(colorImg, depobj);

		cout << "the size of vision_set is:" << vision_set.cntrpoint.size() << endl;
		if (vision_set.cntrpoint.size() == 0)
		{
			cout << "no objects!" << endl;
			vision_set.issuccess = 0;
		}
		for (int i = 0; i < vision_set.cntrpoint.size(); i++)
		{
			vision_set.cntrpoint[i][2] = depImg.at<unsigned short int>(int(vision_set.cntrpoint[i][1]), int(vision_set.cntrpoint[i][0])) * 0.001;
			vision_set.cntrpoint[i][0] = -(vision_set.cntrpoint[i][0] - camera_cx) * vision_set.cntrpoint[i][2] / camera_fx;
			vision_set.cntrpoint[i][1] = (vision_set.cntrpoint[i][1] - camera_cy) * vision_set.cntrpoint[i][2] / camera_fy;

			ROS_INFO("No.%d:  XYZ-theta: %f, %f, %f, %f  Class: %d  Grip_Method: %d", i,
					 vision_set.cntrpoint[i][0], vision_set.cntrpoint[i][1], vision_set.cntrpoint[i][2], vision_set.cntrpoint[i][3],
					 vision_set.classify[i], vision_set.grip_method[i]);
		}
	}

	/***********overlap object detect**************/
	//RNG rng(12345);
	vector<float> overlap(Mat srcImage0, int flag, int depth)
	{
		imwrite("ll.jpg",srcImage0);
		if (flag == 0)
		{
			camera_cx_wrist = 488.904;
			camera_cy_wrist = 280.927;
			camera_fx_wrist = 405.821;
			camera_fy_wrist = 405.821;
		}
		else if (flag == 1)
		{
			camera_cx_wrist = 467.294;
			camera_cy_wrist = 340.350;
			camera_fx_wrist = 404.940;
			camera_fy_wrist = 404.940;
		}

		vector<float> p(4);
		Point final_point;
		float angle;
		final_point = getOverlapPoint(srcImage0, angle);
		p[2] = depth / camera_factor_wrist;
		p[0] = (final_point.x - camera_cx_wrist) * p[2] / camera_fx_wrist;
		p[1] = (final_point.y - camera_cy_wrist) * p[2] / camera_fy_wrist;
		p[3] = angle;
		ROS_INFO("xyz-angle:%f,%f,%f,%f",p[0],p[1],p[2],p[3]);
		return p;
	}

	Point getOverlapPoint(Mat &srcImage0, float &angle)
	{

		Mat srcImage = Mat::zeros(600, 960, CV_8UC3);

		resize(srcImage0, srcImage, srcImage.size());
		cvtColor(srcImage0, srcImage, COLOR_BGR2GRAY);
		//Mat copyImg = srcImage0.clone();
		srcImage = srcImage > 50; //二值化

		imshow("原图", srcImage);
		//getStructuringElement函数会返回指定形状和尺寸的结构元素
		Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
		//morphologyEx(srcImage, srcImage, MORPH_CLOSE, element);//闭运算滤波
		vector<vector<Point>> contours, RectContours;					//轮廓，为点向量，
		findContours(srcImage, contours, RETR_LIST, CHAIN_APPROX_NONE); //找轮廓

		vector<vector<Point>> hull(contours.size()); //用于存放凸包
		Mat drawing(srcImage.size(), CV_8UC3, cv::Scalar(0));
		int i = 0;
		Point2f eigen_vecs;
		Point cntr;
		vector<float> length(contours.size()); //用于保存每个轮廓的长度
		vector<float> Area_contours(contours.size()), Area_hull(contours.size()), Rectangularity(contours.size()), circularity(contours.size());
		for (i = 0; i < contours.size(); i++)
		{ //把所有的轮廓画出来
			Scalar color = Scalar(255, 255, 255);
			length[i] = arcLength(contours[i], true); //轮廓的长度
			//ROS_INFO("No. %d length of the contour:%f",i,length[i]);
			if (length[i] > 450 && length[i] < 850)
			{																				//通过长度匹配滤除小轮廓
				convexHull(Mat(contours[i]), hull[i], false);								//把凸包找出来，寻找凸包函数
				Area_contours[i] = contourArea(contours[i]);								//轮廓面积
				Area_hull[i] = contourArea(hull[i]);										//凸包面积
				Rectangularity[i] = Area_contours[i] / Area_hull[i];						//矩形度
				circularity[i] = (4 * 3.1415 * Area_contours[i]) / (length[i] * length[i]); //圆形度
				//drawContours(drawing, contours, i, color, 1);//得到方框

				//if (Rectangularity[i] > 0.8 && circularity[i] < 0.9)
				//{//通过矩形度和圆形度滤除数字
				//drawContours(drawing, contours, i, Scalar(255, 255, 255), 1);
				RectContours.push_back(hull[i]);		  //把提取出来的方框导入到新的轮廓组
				drawContours(drawing, hull, i, color, 1); //得到方框
				//imshow("drawing", drawing);
				//waitKey(0);
				eigen_vecs = getOrientation(contours[i], srcImage0,cntr); //PCA分析
				angle = atan(-eigen_vecs.y / eigen_vecs.x) * 180 / 3.1415;
				if (angle < 0)
				{
					angle = 180 + angle;
				}

				//}
			}
		}

		float distance = 0, distanceMax = 0;
		Point connorPoint1, connorPoint2, connorPoint3, connorPoint4, point_add;
		vector<Point> connor4_add(3); //先找到的三个角点
		int conP_i1, conP_i2, conP_i3, conP_i_add;
		int j = 0, flag = 0;

		Point2f finally_contours[80][4]; //轮廓，为点向量，新的轮廓
		Point OverlapPoint;
		for (j = 0; j < RectContours.size(); j++) //四边形轮廓个数
		{
			distance = 0;
			distanceMax = 0;
			int k = 0;
			while (k < RectContours[j].size())
			{
				for (i = k; i < RectContours[j].size(); i++) //每个轮廓点的个数11到19点不等
				{											 //找第一个角点
					distance = getDistance(RectContours[j][i], RectContours[j][0]);
					if (distance > distanceMax)
					{
						distanceMax = distance;
						connorPoint1 = RectContours[j][i]; //第一个角点
						conP_i1 = i;
					}
				}
				distance = 0;
				distanceMax = 0;
				for (i = 0; i < RectContours[j].size(); i++)
				{ //找第二个角点
					distance = getDistance(RectContours[j][i], connorPoint1);
					if (distance > distanceMax)
					{
						distanceMax = distance;
						connorPoint2 = RectContours[j][i]; //第二个角点
						conP_i2 = i;
					}
				}
				distance = 0;
				distanceMax = 0;
				for (i = 0; i < RectContours[j].size(); i++)
				{ //找第三个角点
					distance = getDistance(RectContours[j][i], connorPoint1) + getDistance(RectContours[j][i], connorPoint2);
					if (distance > distanceMax)
					{
						distanceMax = distance;
						connorPoint3 = RectContours[j][i]; //第三个角点
						conP_i3 = i;
					}
				}
				flag = list_connor(conP_i1, conP_i2, conP_i3); //对三个角点由大到小排序
				//cout << "conP_i1,2,3--" << conP_i1 << "," << conP_i2 << "," << conP_i3 << endl;
				//cout << "flag=" << flag;
				switch (flag)
				{ //对三个角点排序
				case 0:
					break;
				case 123:
					break;
				case 132:
					point_add = connorPoint2;
					connorPoint2 = connorPoint3;
					connorPoint3 = point_add;
					break; //2,3交换
				case 213:
					point_add = connorPoint1;
					connorPoint1 = connorPoint2;
					connorPoint2 = point_add;
					break; //1,2交换
				case 231:
					point_add = connorPoint1;
					connorPoint1 = connorPoint2;
					connorPoint2 = point_add;
					point_add = connorPoint2;
					connorPoint2 = connorPoint3;
					connorPoint3 = point_add;
					break; //1,2交换+2,3交换
				case 321:
					point_add = connorPoint3;
					connorPoint3 = connorPoint1;
					connorPoint1 = point_add;
					break; //1,3交换
				case 312:
					point_add = connorPoint3;
					connorPoint3 = connorPoint1;
					connorPoint1 = point_add;
					point_add = connorPoint2;
					connorPoint2 = connorPoint3;
					connorPoint3 = point_add;
					break; //1,3交换+2,3交换
				}
				switch (flag)
				{ //对三个角点排序
				case 0:
					break;
				case 123:
					break;
				case 132:
					conP_i_add = conP_i2;
					conP_i2 = conP_i3;
					conP_i3 = conP_i_add;
					break; //2,3交换
				case 213:
					conP_i_add = conP_i1;
					conP_i1 = conP_i2;
					conP_i2 = conP_i_add;
					break; //1,2交换
				case 231:
					conP_i_add = conP_i1;
					conP_i1 = conP_i2;
					conP_i2 = conP_i_add;
					conP_i_add = conP_i2;
					conP_i2 = conP_i3;
					conP_i3 = conP_i_add;
					break; //1,2交换+2,3交换
				case 321:
					conP_i_add = conP_i3;
					conP_i3 = conP_i1;
					conP_i1 = conP_i_add;
					break; //1,3交换
				case 312:
					conP_i_add = conP_i3;
					conP_i3 = conP_i1;
					conP_i1 = conP_i_add;
					conP_i_add = conP_i2;
					conP_i2 = conP_i3;
					conP_i3 = conP_i_add;
					break; //1,3交换+2,3交换
				}
				distance = 0;
				distanceMax = 0;
				for (i = conP_i3; i < conP_i2; i++)
				{ //相隔两角点之间找到怀疑是4角点的点
					distance = getDistance(RectContours[j][i], connorPoint3) + getDistance(RectContours[j][i], connorPoint2);
					if (distance > distanceMax)
					{
						distanceMax = distance;
						connor4_add[0] = RectContours[j][i];
					}
				}
				distance = 0;
				distanceMax = 0;
				for (i = conP_i2; i < conP_i1; i++)
				{ //相隔两角点之间找到怀疑是4角点的点
					distance = getDistance(RectContours[j][i], connorPoint1) + getDistance(RectContours[j][i], connorPoint2);
					if (distance > distanceMax)
					{
						distanceMax = distance;
						connor4_add[1] = RectContours[j][i];
					}
				}
				distance = 0;
				distanceMax = 0;
				for (i = conP_i1; i < RectContours[j].size() + conP_i3; i++)
				{ //相隔两角点之间找到怀疑是4角点的点
					if (i < RectContours[j].size())
					{
						distance = getDistance(RectContours[j][i], connorPoint1) + getDistance(RectContours[j][i], connorPoint3);
						if (distance > distanceMax)
						{
							distanceMax = distance;
							connor4_add[2] = RectContours[j][i];
						}
					}
					else
					{
						distance = getDistance(RectContours[j][i - RectContours[j].size()], connorPoint1) + getDistance(RectContours[j][i - RectContours[j].size()], connorPoint3);
						if (distance > distanceMax)
						{
							distanceMax = distance;
							connor4_add[2] = RectContours[j][i - RectContours[j].size()];
						}
					}
				}
				if (getDist_P2L(connor4_add[0], connorPoint3, connorPoint2) > 10)
				{
					connorPoint4 = connor4_add[0];
				}
				else if (getDist_P2L(connor4_add[1], connorPoint2, connorPoint1) > 10)
				{
					connorPoint4 = connor4_add[1];
				}
				else if (getDist_P2L(connor4_add[2], connorPoint1, connorPoint3) > 10)
				{
					connorPoint4 = connor4_add[2];
				}

				//circle(drawing, connorPoint1, 3, Scalar(255, 255, 255), FILLED, LINE_AA);
				//circle(drawing, connorPoint2, 3, Scalar(255, 255, 255), FILLED, LINE_AA);
				//circle(drawing, connorPoint3, 3, Scalar(255, 255, 255), FILLED, LINE_AA);
				//circle(drawing, connorPoint4, 3, Scalar(255, 255, 255), FILLED, LINE_AA);

				finally_contours[j][0] = connorPoint1;
				finally_contours[j][1] = connorPoint2;
				finally_contours[j][2] = connorPoint3;
				finally_contours[j][3] = connorPoint4;

				//cout << "\n轮廓 " << j + 1 << "  的四个角点坐标分别为：\n" << finally_contours[j][0] << finally_contours[j][1] << finally_contours[j][2] << finally_contours[j][3] << endl;

				//找到距离最近的两点---------------------------------
				float mindistance;
				int m1 = 0, m2 = 1;
				mindistance = getDistance(finally_contours[j][0], finally_contours[j][1]);
				for (int n = 0; n < 4; n++)
				{
					for (int m = n + 1; m < 4; m++)
					{
						float distance1 = getDistance(finally_contours[j][n], finally_contours[j][m]);
						if (distance1 < mindistance)
						{
							mindistance = distance1;
							m1 = n;
							m2 = m;
						}
					}
				}
				int m3 = 0, m4 = 0;
				for (int n = 0; n < 4; n++)
				{
					if ((n != m1) & (n != m2))
					{
						m3 = n;
					}
				}
				for (int n = 0; n < 4; n++)
				{
					if ((n != m1) & (n != m2) & (n != m3))
					{
						m4 = n;
					}
				}

				//cout << "距离最近的两点坐标分别为：" << finally_contours[j][m1] << finally_contours[j][m2] << endl;
				//cout << "其余两点坐标分别为：" << finally_contours[j][m3] << finally_contours[j][m4] << endl;

				//PCA方法找到长轴----------------------------------
				Point real_finally_contours[2];
				real_finally_contours[0].x = (finally_contours[j][m1].x + finally_contours[j][m2].x) / 2.0;
				real_finally_contours[0].y = (finally_contours[j][m1].y + finally_contours[j][m2].y) / 2.0;
				real_finally_contours[1].x = (finally_contours[j][m3].x + finally_contours[j][m4].x) / 2.0;
				real_finally_contours[1].y = (finally_contours[j][m3].y + finally_contours[j][m4].y) / 2.0;
				//cout << "距离最近的两点中点为：" << real_finally_contours[0] << endl;
				//cout << "其余两点中点坐标为：" << real_finally_contours[1] << endl;

				//找到交点
				vector<Vec4i> lines(2);
				lines[0][0] = real_finally_contours[0].x - 1000 * eigen_vecs.x;
				lines[0][1] = real_finally_contours[0].y - 1000 * eigen_vecs.y;
				lines[0][2] = real_finally_contours[0].x + 1000 * eigen_vecs.x;
				lines[0][3] = real_finally_contours[0].y + 1000 * eigen_vecs.y;
				lines[1][0] = finally_contours[j][m3].x;
				lines[1][1] = finally_contours[j][m3].y;
				lines[1][2] = finally_contours[j][m4].x;
				lines[1][3] = finally_contours[j][m4].y;

				Point pt = getCrossPoint(lines[0], lines[1]);

				//交点与距离最近两点中点的中点，即所求抓取点

				OverlapPoint.x = (real_finally_contours[0].x + pt.x) / 2.0;
				OverlapPoint.y = (real_finally_contours[0].y + pt.y) / 2.0;
				OverlapPoint.x = (OverlapPoint.x + cntr.x * 2) / 3.0;
				OverlapPoint.y = (OverlapPoint.y + cntr. y * 2) / 3.0;
				//cout << "所求抓取点为：" << OverlapPoint << endl;
				//circle(drawing, OverlapPoint, 3, Scalar(0, 0, 255), 2);
				if (cv::pointPolygonTest(RectContours[j], OverlapPoint, false) == 1)
				{
					break;
				}
				k = k + 3;
			}
		}
		circle(srcImage0 /*drawing*/, connorPoint1, 3, Scalar(255, 255, 255), FILLED, LINE_AA);
		circle(srcImage0 /*drawing*/, connorPoint2, 3, Scalar(255, 255, 255), FILLED, LINE_AA);
		circle(srcImage0 /*drawing*/, connorPoint3, 3, Scalar(255, 255, 255), FILLED, LINE_AA);
		circle(srcImage0 /*drawing*/, connorPoint4, 3, Scalar(255, 255, 255), FILLED, LINE_AA);
		//cout << "所求抓取点为：" << OverlapPoint << endl;
		circle(srcImage0 /*drawing*/, OverlapPoint, 3, Scalar(0, 0, 255), 2);
		imshow("contour", srcImage0 /*drawing*/);
		cv::waitKey(1000);
		//system("pause");
		return OverlapPoint;
	}

	float getDist_P2L(Point pointP, Point pointA, Point pointB)
	{ //点到直线的距离:P到AB的距离
		//求直线方程
		int A = 0, B = 0, C = 0;
		A = pointA.y - pointB.y;
		B = pointB.x - pointA.x;
		C = pointA.x * pointB.y - pointA.y * pointB.x;
		//代入点到直线距离公式
		float distance = 0;
		distance = ((float)abs(A * pointP.x + B * pointP.y + C)) / ((float)sqrtf(A * A + B * B));
		return distance;
	}

	//对角点进行排序，因为之前检测出的轮廓是带序号的
	int list_connor(int i1, int i2, int i3)
	{ //排序
		int flag = 0;
		Point point_add;
		if (i1 >= i2 && i2 >= i3)
			flag = 123;
		else if (i1 >= i3 && i3 >= i2)
			flag = 132;
		else if (i2 >= i1 && i1 >= i3)
			flag = 213;
		else if (i2 >= i3 && i3 >= i1)
			flag = 231;
		else if (i3 >= i2 && i2 >= i1)
			flag = 321;
		else if (i3 >= i1 && i1 >= i2)
			flag = 312;
		return flag;
	}

	float getDistance(Point pointO, Point pointA)
	{ //求两点之间距离
		float distance;
		distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
		distance = sqrtf(distance);

		return distance;
	}

	float getAngle(CvPoint pointM, CvPoint pointL, CvPoint pointR)
	{ //求三点之间的夹角
		CvPoint L, R;
		float dist_L, dist_R, Theta;
		L.x = pointL.x - pointM.x;
		L.y = pointL.y - pointM.y;
		R.x = pointR.x - pointM.x;
		R.y = pointR.y - pointM.y;
		dist_L = getDistance(pointL, pointM);
		dist_R = getDistance(pointR, pointM);
		Theta = acos((L.x * R.x + L.y * R.y) / (dist_L * dist_R));
		return Theta;
	}

	Point2f getOrientation(const vector<Point> &pts, Mat &img, Point &cntr)
	{
		// 构造一个用于pca分析的缓冲区
		int sz = static_cast<int>(pts.size());
		Mat data_pts = Mat(sz, 2, CV_64F);
		for (int i = 0; i < data_pts.rows; i++)
		{
			data_pts.at<double>(i, 0) = pts[i].x;
			data_pts.at<double>(i, 1) = pts[i].y;
		}
		// 进行PCA分析
		PCA pca_analysis(data_pts, Mat(), PCA::DATA_AS_ROW);
		// 存储物体中心
		cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
						   static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
		// 存储特征值和特征向量
		//vector<Point2d> eigen_vecs(2);
		vector<Point2d> eigen_vecs(2);
		vector<double> eigen_val(2);
		for (int i = 0; i < 2; i++)
		{
			eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
									pca_analysis.eigenvectors.at<double>(i, 1));
			eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
		}
		// 绘制主要成分
		circle(img, cntr, 3, Scalar(255, 0, 255), 2);
		Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
		Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
		drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
		drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
		double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // 弧度方向
		Point2f crossPoint;
		crossPoint.x = eigen_vecs[0].x;
		crossPoint.y = eigen_vecs[0].y;
		return crossPoint;
	}

	void drawAxis(Mat &img, Point p, Point q, Scalar colour, const float scale = 0.2)
	{
		double angle = atan2((double)p.y - q.y, (double)p.x - q.x); // angle in radians
		double hypotenuse = sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
		// 将箭头加长了一定比例
		q.x = (int)(p.x - scale * hypotenuse * cos(angle));
		q.y = (int)(p.y - scale * hypotenuse * sin(angle));
		line(img, p, q, colour, 1, LINE_AA);
		// 创建箭头
		p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
		p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
		line(img, p, q, colour, 1, LINE_AA);
		p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
		p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
		line(img, p, q, colour, 1, LINE_AA);
	}

	Point2f getCrossPoint(Vec4i LineA, Vec4i LineB)
	{
		double ka, kb;
		ka = (double)(LineA[3] - LineA[1]) / (double)(LineA[2] - LineA[0]); //求出LineA斜率
		kb = (double)(LineB[3] - LineB[1]) / (double)(LineB[2] - LineB[0]); //求出LineB斜率

		Point2f crossPoint;
		crossPoint.x = (ka * LineA[0] - LineA[1] - kb * LineB[0] + LineB[1]) / (ka - kb);
		crossPoint.y = (ka * kb * (LineA[0] - LineB[0]) + ka * LineB[1] - kb * LineA[1]) / (ka - kb);
		return crossPoint;
	}
	/************************************/

	const int caijian_x = 500, caijian_y = 500, caijian_w = 1000, caijian_h = 400;

	vector<float> bottle_detect(Mat &colorImg, Mat &depthImg, int flag)
	{
		vector<Point> approx;
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		vector<Rect> boundRect(contours.size());
		Mat imgThresholded, imgThresholded_cup;
		Mat imgHSV, imgHSV_bottle;
		Point2f center;
		float radius;
		vector<float> results;
		results.resize(4);

		vector<float> results1;
		results1.resize(4);

		Rect rect1(caijian_x, caijian_y, caijian_w, caijian_h);
		colorImg = colorImg(rect1);
		// imshow("colorImg",colorImg);
		cvtColor(colorImg, imgHSV, COLOR_BGR2HSV);
		Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
		Mat element1 = getStructuringElement(MORPH_RECT, Size(6, 6));
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		dilate(imgThresholded, imgThresholded, element);
		erode(imgThresholded, imgThresholded, element1);
		imshow("Original_cup_detect", imgThresholded);
		results = counter_detect(imgThresholded, colorImg, depthImg);
		//cout<< "results"<<results<< endl;
		if (flag == 1)
		{
			return results;
		}
		//cout<<"res"<<results<<endl;
		cvtColor(colorImg, imgHSV_bottle, COLOR_BGR2HSV);
		// imshow("colorImg1",colorImg);

		inRange(imgHSV_bottle, Scalar(iLowH_cup, iLowS_cup, iLowV_cup), Scalar(iHighH_cup, iHighS_cup, iHighV_cup), imgThresholded_cup); //Threshold the image
		//imshow("imgThresholded1",imgThresholded_cup);

		dilate(imgThresholded_cup, imgThresholded_cup, element);
		erode(imgThresholded_cup, imgThresholded_cup, element1);
		imshow("imgThresholded1", imgThresholded_cup);
		waitKey(100);

		results = counter_detect1(imgThresholded_cup, colorImg, depthImg);
		cout << "res0: " << results[0] << "res1: " << results[1] << "res2: " << results[2] << "res3: " << results[3] << endl;

		return results;
	}

	vector<float> cup_detect(Mat &colorImg, Mat &depthImg)
	{
		vector<float> results;
		Mat imgThresholded_cup;
		Point2f center;
		float radius;
		results.resize(4);
		vector<Mat> hsvSplit_cup;
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;

		cvtColor(colorImg, colorImg, COLOR_BGR2HSV);

		inRange(colorImg, Scalar(iLowH_cup_move, iLowS_cup_move, iLowV_cup_move), Scalar(iHighH_cup_move, iHighS_cup_move, iHighV_cup_move), imgThresholded_cup); //Threshold the image
		Mat element_cup = getStructuringElement(MORPH_RECT, Size(3, 3));
		Mat element1_cup = getStructuringElement(MORPH_RECT, Size(6, 6));
		erode(imgThresholded_cup, imgThresholded_cup, element_cup);
		dilate(imgThresholded_cup, imgThresholded_cup, element1_cup);
		imshow("cup_detect", imgThresholded_cup);
		results = counter_detect(imgThresholded_cup, colorImg, depthImg);
		return results;
	}

	vector<float> counter_detect(Mat &imgThresholded, Mat &colorImg, Mat &depthImg)
	{
		// ROS_INFO("COUNTER DETECT START");
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		vector<Rect> boundRect(contours.size());
		vector<float> results;
		settled_z = 1;
		results.resize(4);
		Point2f center;
		float radius;
		int num = 0;
		double dep_res = 0;
		findContours(imgThresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); i++)
		{
			// ROS_INFO("area: %f", g_dConArea);
			g_dConArea = fabs(contourArea(contours[i], true));
			if (g_dConArea > 3000.0 && g_dConArea < 16000.0)
			{
				ROS_INFO("Couter_squre: %f", g_dConArea);

				Rect boundRect;
				boundRect = boundingRect(Mat(contours[i]));
				//imshow("2",imgThresholded);
				ROS_INFO("boundRect.width: %d,%d", boundRect.x, boundRect.y);
				rectangle(imgThresholded, boundRect, Scalar(255, 255, 255));
				imshow("rectangle", imgThresholded);

				for (int i = boundRect.x + caijian_x; i < boundRect.x + boundRect.width + caijian_x; i++)
				{
					for (int j = boundRect.y + caijian_y; j < boundRect.y + boundRect.height + caijian_y; j++)
					{
						//ROS_INFO("depth: %f", depthImg.at<float>(int(j), int(i))*0.001);
						if (depthImg.at<float>(int(j), int(i)) * 0.001 > 0.4 && depthImg.at<float>(int(j), int(i)) * 0.001 < 1.1)
						{
							dep_res = depthImg.at<float>(int(j), int(i)) * 0.001 + dep_res;
							//ROS_INFO("z: %f", dep_res);
							num = num + 1;
						}
					}
				}

				settled_z = dep_res / num;
				//ROS_INFO("settled_z: %f", settled_z);

				imshow("3", imgThresholded);

				avgX = (boundRect.x + boundRect.x + boundRect.width) / 2;
				avgY = (boundRect.y + boundRect.y + boundRect.height) / 2;
				if (avgX >= 550)
				{
					avgX = avgX - 20;
					avgY = avgY + 15;
				}
				else if (avgX < 550)
				{
					avgX = avgX + 15;
					avgY = avgY + 15;
				}
				avgX = avgX + caijian_x;
				avgY = avgY + caijian_y;

				ROS_INFO("detected_avg: %f, %f,%f", avgX, avgY, settled_z);

				minEnclosingCircle(Mat(contours[i]), center, radius);
				avgX_cir = center.x;
				avgY_cir = center.y;

				//settled_z = depthImg.at<float>(int(avgY), int(avgX))*0.001;

				avgX_cir = avgX_cir + caijian_x;
				avgY_cir = avgY_cir + caijian_y;

				avgX_res = -(avgX - camera_cx) * settled_z / camera_fx;
				avgY_res = (avgY - camera_cy) * settled_z / camera_fy;

				results[0] = avgX_res;
				results[1] = avgY_res;
				results[2] = settled_z;

				ROS_INFO("detected: %f, %f, %f", avgX_res, avgY_res, settled_z);

				Point root_points[1][4];
				root_points[0][0] = Point2i(boundRect.x, boundRect.y);
				root_points[0][1] = Point2i(boundRect.x + boundRect.width, boundRect.y);
				root_points[0][2] = Point2i(boundRect.x + boundRect.width, boundRect.y + boundRect.height);
				root_points[0][3] = Point2i(boundRect.x, boundRect.y + boundRect.height);
				const Point *ppt[1] = {root_points[0]};
				int npt[] = {4};
				polylines(colorImg, ppt, npt, 1, 1, Scalar(0, 0, 0), 1, 8, 0);
				fillPoly(colorImg, ppt, npt, 1, Scalar(0, 0, 0));
				// imshow("rect",colorImg);
				// waitKey(1000);

			}
		}
			ROS_INFO("results: %f, %f, %f", results[0], results[1], results[2]);
			// ROS_INFO("COUNTER DETECT END");
			return results;
	}



	vector<float> counter_detect1(Mat &imgThresholded, Mat &colorImg, Mat &depthImg)
	{
		// ROS_INFO("COUNTER DETECT START");
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		vector<Rect> boundRect(contours.size());
		vector<float> results;
		settled_z = 1;
		results.resize(4);
		Point2f center;
		float radius;
		int num = 0;
		double dep_res = 0;
		findContours(imgThresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); i++)
		{
			// ROS_INFO("area: %f", g_dConArea);
			g_dConArea = fabs(contourArea(contours[i], true));
			if (g_dConArea > 3000.0 && g_dConArea < 16000.0)
			{
				ROS_INFO("Couter_squre1: %f", g_dConArea);

				Rect boundRect;
				boundRect = boundingRect(Mat(contours[i]));
				//imshow("2",imgThresholded);
				ROS_INFO("boundRect.width1: %d,%d", boundRect.x, boundRect.y);
				rectangle(imgThresholded, boundRect, Scalar(255, 255, 255));
				imshow("rectangle1", imgThresholded);

				for (int i = boundRect.x + caijian_x; i < boundRect.x + boundRect.width + caijian_x; i++)
				{
					for (int j = boundRect.y + caijian_y; j < boundRect.y + boundRect.height + caijian_y; j++)
					{
						//ROS_INFO("depth: %f", depthImg.at<float>(int(j), int(i))*0.001);
						if (depthImg.at<float>(int(j), int(i)) * 0.001 > 0.4 && depthImg.at<float>(int(j), int(i)) * 0.001 < 1.1)
						{
							dep_res = depthImg.at<float>(int(j), int(i)) * 0.001 + dep_res;
							//ROS_INFO("z: %f", dep_res);
							num = num + 1;
						}
					}
				}

				settled_z = dep_res / num;
				//ROS_INFO("settled_z: %f", settled_z);

				imshow("31", imgThresholded);

				avgX = (boundRect.x + boundRect.x + boundRect.width) / 2;
				avgY = (boundRect.y + boundRect.y + boundRect.height) / 2;
				if (avgX >= 550)
				{
					avgX = avgX - 20;
					avgY = avgY + 15;
				}
				else if (avgX < 550)
				{
					avgX = avgX + 15;
					avgY = avgY + 15;
				}
				avgX = avgX + caijian_x;
				avgY = avgY + caijian_y;

				ROS_INFO("detected_avg1: %f, %f,%f", avgX, avgY, settled_z);

				minEnclosingCircle(Mat(contours[i]), center, radius);
				avgX_cir = center.x;
				avgY_cir = center.y;

				//settled_z = depthImg.at<float>(int(avgY), int(avgX))*0.001;

				avgX_cir = avgX_cir + caijian_x;
				avgY_cir = avgY_cir + caijian_y;

				avgX_res = -(avgX - camera_cx) * settled_z / camera_fx;
				avgY_res = (avgY - camera_cy) * settled_z / camera_fy;

				results[0] = avgX_res;
				results[1] = avgY_res;
				results[2] = settled_z;

				ROS_INFO("detected1: %f, %f, %f", avgX_res, avgY_res, settled_z);

				Point root_points[1][4];
				root_points[0][0] = Point2i(boundRect.x, boundRect.y);
				root_points[0][1] = Point2i(boundRect.x + boundRect.width, boundRect.y);
				root_points[0][2] = Point2i(boundRect.x + boundRect.width, boundRect.y + boundRect.height);
				root_points[0][3] = Point2i(boundRect.x, boundRect.y + boundRect.height);
				const Point *ppt[1] = {root_points[0]};
				int npt[] = {4};
				polylines(colorImg, ppt, npt, 1, 1, Scalar(0, 0, 0), 1, 8, 0);
				fillPoly(colorImg, ppt, npt, 1, Scalar(0, 0, 0));
				// imshow("rect",colorImg);
				// waitKey(1000);

			}
		}
			ROS_INFO("results1: %f, %f, %f", results[0], results[1], results[2]);
			// ROS_INFO("COUNTER DETECT END");
			return results;
	}

		vector<float> hand_counter_detect(Mat & imgThresholded, int flag)
		{
			vector<vector<Point>> contours;
			vector<Vec4i> hierarchy;
			vector<Rect> boundRect(contours.size());
			vector<float> results;
			results.resize(4);
			settled_z = 10;
			Point2f center;
			float radius;
			if (flag == 0)
			{
				camera_cx_wrist = 488.904;
				camera_cy_wrist = 280.927;
				camera_fx_wrist = 405.821;
				camera_fy_wrist = 405.821;
			}
			else if (flag == 1)
			{
				camera_cx_wrist = 467.294;
				camera_cy_wrist = 340.350;
				camera_fx_wrist = 404.940;
				camera_fy_wrist = 404.940;
			}
			findContours(imgThresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			for (int i = 0; i < contours.size(); i++)
			{
				g_dConArea = fabs(contourArea(contours[i], true));
				ROS_INFO("g_dConArea_CUP: %f", g_dConArea);
				if (g_dConArea > 10000 && g_dConArea < 30000)
				{
					Rect boundRect;
					boundRect = boundingRect(Mat(contours[i]));
					rectangle(imgThresholded, boundRect, Scalar(255, 255, 255));
					imshow("hand_counter_detec1", imgThresholded);
					//waitKey(1000);
					avgX = (boundRect.x + boundRect.x + boundRect.width) / 2;
					avgY = (boundRect.y + boundRect.y + boundRect.height) / 2;
					ROS_INFO("bottle_Square: %d", boundRect.height * boundRect.width);

					if (float(boundRect.width) / float(boundRect.height) > 0.8 && float(boundRect.width) / float(boundRect.height) < 1.2)
					{
						ROS_INFO("detected_avg: %f, %f", avgX, avgY);
						minEnclosingCircle(Mat(contours[i]), center, radius);
						avgX_cir = center.x;
						avgY_cir = center.y;
						settled_z = 0.2;
						avgX_res = (avgX_cir - camera_cx_wrist) * settled_z / camera_fx_wrist;
						avgY_res = (avgY_cir - camera_cy_wrist) * settled_z / camera_fx_wrist;

						results[0] = avgX_res;
						results[1] = avgY_res;
						results[2] = settled_z;
					}

				}
			}
			return results;
		}

		vector<float> hand_cup_detect(Mat & colorImg, int flag)
		{
			ROS_INFO("Hand_cup_detect start");
			vector<float> results;
			Mat imgThresholded_cup;

			results.resize(4);
			vector<Mat> hsvSplit_cup;
			vector<vector<Point>> contours;
			vector<Vec4i> hierarchy;
			cvtColor(colorImg, colorImg, COLOR_BGR2HSV);
			inRange(colorImg, Scalar(iLowH_hand_cup, iLowS_hand_cup, iLowV_hand_cup), Scalar(iHighH_hand_cup, iHighS_hand_cup, iHighV_hand_cup), imgThresholded_cup); //Threshold the image
			Mat element_cup = getStructuringElement(MORPH_RECT, Size(3, 3));
			Mat element1_cup = getStructuringElement(MORPH_RECT, Size(6, 6));

			erode(imgThresholded_cup, imgThresholded_cup, element_cup);
			dilate(imgThresholded_cup, imgThresholded_cup, element1_cup);
			imshow("hand_cup_detect", imgThresholded_cup);

			Point2f center;
			float radius;
			results = hand_counter_detect(imgThresholded_cup, flag);
			//waitKey(1000);
			ROS_INFO("Hand_cup_detect end");
			ROS_INFO("results_handcup: %f, %f, %f", results[0], results[1], results[2]);
			return results;
		}

		vector<float> hand_bottle_detect(Mat & colorImg, int flag)
		{
			ROS_INFO("Hand_bottle_detect start");
			vector<float> results;
			Mat imgThresholded_bottle;
			results.resize(4);
			vector<Mat> hsvSplit_cup;
			vector<vector<Point>> contours;
			vector<Vec4i> hierarchy;
			// imshow("hand_bottle_color", colorImg);
			cvtColor(colorImg, colorImg, COLOR_BGR2HSV);

			inRange(colorImg, Scalar(iLowH_hand_bottle, iLowS_hand_bottle, iLowV_hand_bottle), Scalar(iHighH_hand_bottle, iHighS_hand_bottle, iHighV_hand_bottle), imgThresholded_bottle); //Threshold the image
			Mat element_cup = getStructuringElement(MORPH_RECT, Size(3, 3));
			Mat element1_cup = getStructuringElement(MORPH_RECT, Size(6, 6));
			//imshow("hand_bottle_detect", imgThresholded_bottle);
			erode(imgThresholded_bottle, imgThresholded_bottle, element_cup);
			dilate(imgThresholded_bottle, imgThresholded_bottle, element1_cup);
			Point2f center;
			float radius;
			results = hand_counter_detect(imgThresholded_bottle, flag);
			imshow("hand_cup_detect", imgThresholded_bottle);
			ROS_INFO("Hand_bottle_detect end");
			return results;
		}
	};

	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "image_process");
		ros::NodeHandle nh;

		ImgProc imageProcess(nh);

		while (ros::ok())
		{
			ros::spin();
		}
	}
