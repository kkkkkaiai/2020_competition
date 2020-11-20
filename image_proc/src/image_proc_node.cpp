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

const int MinArea = 150;
const int MaxArea = 600;

int suppres_min = 955, suppres_max = 1180;
int spprs_xmin = 500, spprs_xmax = 1570, spprs_ymin = 500, spprs_ymax = 880;
int MinArea_vtc = 40, MaxArea_vtc = 180;
int MinArea_clr = 90, MaxArea_clr = 500;
int MinArea_clr_hori = 100, MaxArea_clr_hori = 350;
int gray_supp = 120;

#define ver_dep_min 40
#define ver_dep_max 85

struct vision
{
	bool issuccess;
	vector<int> grip_method;		 //1--vertical,0--horizontal
	vector<vector<float>> cntrpoint; //xyz and angel of center point
	vector<int> classify;			 //the classification of the object
};

vision vision_set;

//save image cnt
int cnt=0;

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

	cv_bridge::CvImagePtr left_cv_Img_ptr;
	cv_bridge::CvImagePtr right_cv_Img_ptr;

	Mat left_color_image;
	Mat right_color_image;

public:
	ImgProc(ros::NodeHandle &nh)
		: nh_(nh), it_(nh)
	{
		server_ = nh_.advertiseService("Image_Process_A", &ImgProc::imageProcessCallback, this);
		sub_left_cam = it_.subscribe("/cameras/left_hand_camera/image", 1, &ImgProc::lWristImgProcessCallbk, this);
		sub_right_cam = it_.subscribe("/cameras/right_hand_camera/image", 1, &ImgProc::rWristImgProcessCallbk, this);

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

		

		vector<float> result;
		bool vision_set_judge = false;
		bool taskA_fun1_judge = false;

		try
		{   
			if (req.method == -1){
				depthimg_save(rgbd2, rgbmat);
				return true;
			}else{
				switch (req.method)
				{
					case 0 : result = single(rgbd2, rgbmat);break;
					case 1 : result = wristRecog(left_color_image, 1, 215);break;
					case 2 : result = wristRecog(right_color_image, 0, 215);break;
					case 4 : result = wristRecog(left_color_image, 1, 185);break;
					case 5 : result = wristRecog(right_color_image, 0, 185);break; // task B
					default: vision_set_judge = true;
				}
				if(vision_set_judge){
					switch (req.method)
					{
						case 3 : taskA_fun1(rgbmat, rgbd2);ROS_INFO("3");break;
						default: taskA_fun1_judge = true;
					}
					if(taskA_fun1_judge){
						ROS_ERROR("Request Method ERROR");
						return false;
					}else{
						resp.is_success = vision_set.issuccess;
						resp.grip_method = vision_set.grip_method;
						for(auto i: vision_set.cntrpoint){
							image_proc_msgs::ImageProcMsg temp;
							vector<float> pos(i.begin(), i.begin()+3);
							vector<float> orient(i.begin()+3, i.end());
							temp.position = pos;
							temp.orientation = orient;

							resp.object.push_back(temp);
						}
						resp.classify = vision_set.classify;
					}
				}else{
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
		}
		catch (ros::Exception e)
		{
			ROS_ERROR("Image process fail");
			return false;
		}

		return true;
	}

	//only use to save Image
	bool depthimg_save(const cv::Mat &depimg,cv::Mat colorImg/*, std::string path*/)
	{
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
		cv::imwrite("./"+to_string(cnt)+".bmp", saveimg);
		cv::imwrite("./"+to_string(cnt)+".jpg", colorImg);
		cnt++;
		return true;
	}

	//use in code
	bool depthimg_save2(const cv::Mat& depimg, std::string path)
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
            	saveimg.at<Vec3b>(j,i)[0]= int(depimg.at<float>(j,i))%256;
				saveimg.at<Vec3b>(j,i)[1]= int(depimg.at<float>(j,i)/256);
				saveimg.at<Vec3b>(j,i)[2]= 0;
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

	vector<float> wristRecog(Mat colorImg, int flag, float depth)
	{
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
		threshold(grayImg, binary, 80, 255, THRESH_BINARY);
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
			if (contours[i].size() > MinArea && contours[i].size() < MaxArea)
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

		p[2] = depth / camera_factor * 0.001;
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

	void findvertical(Mat depImg, vector<Point2f> &crosspcnt)
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

		cv::imshow("binary", binary);
		cv::waitKey(1000);
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

				//circle(binary, center, 3, Scalar(180), -1);
				//cout << "(" << center.x << "," << center.y << ")"
				//	<< "     ";
				//cout << contours[i].size() << endl;
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

	int predict(string path)
	{
		Ptr<ml::SVM> svm = ml::SVM::load("/home/ljq/ros_ws/src/image_proc/param/svm.xml");
		Mat image = imread(path);
		Mat img = image.reshape(1, 1);
		img.convertTo(img, CV_32FC1);
		//cout << int(svm->predict(img)) << endl;
		return int(svm->predict(img));
	}

	void datapadding(Mat &roi)
	{
		if (roi.type() != CV_8UC1)
		{
			cout << "input error!" << endl;
			return;
		}
		int n = 70;
		int pixmin = 50;
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

	void rotate_gjx(Mat image, float angle, Point2f center, Point2f vertex[], std::string filename)
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
			if (contours[i].size() > MinArea_clr && contours[i].size() < MaxArea_clr)
			{

				Moments M = moments(contours[i]);
				Point2f cenMsk;
				cenMsk.x = M.m10 / M.m00;
				cenMsk.y = M.m01 / M.m00;
				//œØÈ¡
				Mat roi = Mat(img_rot, boundRect[i]);
				Mat temp;
				cvtColor(roi, temp, COLOR_BGR2GRAY);
				datapadding(temp);
				/*imshow("roi", roi);
		waitKey(1000);*/
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

		cv::imshow("binary", binaryimg);
		cv::waitKey(1000);

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
				if (contours[i].size() > MinArea_clr && contours[i].size() < MaxArea_clr)
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
							cout << "angle:" << angle << endl;
							temp.push_back(angle);

							vision_set.cntrpoint.push_back(temp);

							Point2f vertex[4];
							boundRect[i].points(vertex);
							std::string filename;
							filename = "./o.jpg";
							rotate_gjx(colorImg, -angle, center, vertex, filename);

							cout << "class:" << predict(filename) << endl;

							vision_set.classify.push_back(predict(filename));

							//visualization
							for (int j = 0; j < 4; j++)
							{
								line(copyimg, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255));
							}
							drawContours(copyimg, contours, i, Scalar(0, 0, 255), 1);
							circle(copyimg, center, 3, Scalar(0, 0, 255), -1);
							cout << "(" << center.x << "," << center.y << ")"
								 << "     ";
							cout << contours[i].size() << endl;

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
			cv::imshow("colorImg", colorImg);
			cv::waitKey(1000);
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

		//cv::imshow("binary", binaryimg);
		//cv::waitKey(1000);

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
				if (contours[i].size() > MinArea_clr && contours[i].size() < MaxArea_clr)
				{
					vision_set.issuccess = 1;
					vision_set.grip_method.push_back(0);

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
					cout << "(" << center.x << "," << center.y << ")"
						 << "     ";
					cout << "contours size:" << contours[i].size() << endl;

					double angle;
					if (boundRect[i].size.height > boundRect[i].size.width)
						angle = 90.0 + abs(boundRect[i].angle);
					else
						angle = abs(boundRect[i].angle);

					cout << "angle:" << angle << endl;
					cout << endl;

					vector<float> temp;
					temp.push_back(center.x);
					temp.push_back(center.y);
					temp.push_back(1000);
					temp.push_back(angle);
					vision_set.cntrpoint.push_back(temp);

					vision_set.classify.push_back(-1);
				}
			}
			cv::imshow("result", copyimg);
			cv::waitKey(1000);
		}
	}

	void find_stand_obj(Mat colorImg, Mat depobj)
	{
		vector<Point2f> crosspcnt;
		findvertical(depobj, crosspcnt);
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
		find_stand_obj(colorImg, depobj);
		findhorizontal(colorImg, depobj);
		cout << "the size of vision_set is:" << vision_set.cntrpoint.size() << endl;
		if (vision_set.cntrpoint.size() == 0)
		{
			cout << "no objects!" << endl;
			vision_set.issuccess = 0;
		}
		for (int i = 0; i < vision_set.cntrpoint.size(); i++)
		{
			vision_set.cntrpoint[i][2] = depImg.at<unsigned short int>(int(vision_set.cntrpoint[i][1]), int(vision_set.cntrpoint[i][0]))*0.001;
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
