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
#include <image_proc_msgs/ImageProcSrv.h>
#include <image_proc_msgs/ImageProcMsg.h>
 
#define DEBUG_MODE
using namespace std;
using namespace cv;
 
enum
{
	Processor_cl,
	Processor_gl,
	Processor_cpu
};
 
const float camera_factor = 1;
const float camera_cx = 960;
const float camera_cy = 540;
//const float camera_fx = 1.0531094819393613e+03;
//const float camera_fy = 1.0531094819393613e+03;
const float camera_fx = 1.081e+03;
const float camera_fy = 1.081e+03;


class ImgProc{
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = NULL;
	libfreenect2::PacketPipeline  *pipeline = NULL;
    string serial;
    int depthProcessor;
    libfreenect2::SyncMultiFrameListener *listener = NULL;
    libfreenect2::Registration* registration = NULL;
    libfreenect2::FrameMap frames;
    Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
	
	ros::NodeHandle nh_; 
    ros::ServiceServer server_;

public:
    ImgProc(ros::NodeHandle nh)
	: nh_(nh) 
	{
        server_ = nh_.advertiseService("Image_Process_A", &ImgProc::imageProcessCallback, this);

        if(freenect2.enumerateDevices() == 0)
	    {
            std::cout << "no device connected!" << std::endl;
            return;
	    }
        serial = freenect2.getDefaultDeviceSerialNumber();
        std::cout << "SERIAL: " << serial << std::endl;
        int depthProcessor = Processor_gl;
        if(depthProcessor == Processor_cpu)
        {
            if(!pipeline)
                //! [pipeline]
                pipeline = new libfreenect2::CpuPacketPipeline();
                cout << "choose cpu mode" << endl;
            //! [pipeline]
        }
        else if (depthProcessor == Processor_gl) // if support gl
        {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
            if(!pipeline)
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
            if(!pipeline)
                pipeline = new libfreenect2::OpenCLPacketPipeline();
                cout << "choose cl mode" << endl;
#else
            std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
        }
        if(pipeline)
        {
            dev = freenect2.openDevice(serial, pipeline);
        }
        else
        {
            dev = freenect2.openDevice(serial);
        }
        if(dev == 0)
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
        
    }

    ~ImgProc()
    {
        dev->stop();
	    dev->close();
        delete listener;  
        delete pipeline;
        delete registration;  
    }

    bool imageProcessCallback(image_proc_msgs::ImageProcSrv::Request &req,
							  image_proc_msgs::ImageProcSrv::Response &resp){
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
		// cv::waitKey(0);
		ROS_INFO("call success");
		image_proc_msgs::ImageProcMsg temp;

		vector<float> result = single(rgbd2,rgbmat);
		vector<float> pos(result.begin(), result.begin()+3);
		vector<float> orient(result.begin()+3, result.end());
		temp.position = pos;
		temp.orientation = orient;
		resp.object.resize(1);
		resp.object[0] = temp;
		ROS_INFO("ImageProcess handle success!");
    }
	
	bool depthimg_save2(const cv::Mat& depimg, std::string path)
	{
    //×¢Òâ£¡£¡ÎÄŒþÃûÒ»¶šÒªŽæ³ÉbmpžñÊœ£¡£¡²»È»»á³öŽí
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

	void singleobject(Mat colorImg, Mat depImg, float& angle, Point2d& center)
	{		
		int spprs_xmin = 480, spprs_xmax = 1600, spprs_ymin = 420, spprs_ymax = 850;
		int MinArea_clr = 100, MaxArea_clr = 500;
		int graysupp=50;
		Mat grayImg;

		Mat copyimg = colorImg.clone();
		Mat binaryimg = Mat::zeros(colorImg.rows, colorImg.cols, CV_8UC1);
		ROS_INFO("1");
		cvtColor(colorImg, grayImg, COLOR_BGRA2GRAY);

		imwrite("./gray.jpg",grayImg);
		depthimg_save2(depImg, "./depth.bmp");

		ROS_INFO("3");
		for (int i = spprs_xmin; i < spprs_xmax; i = i + 1)
			for (int j = spprs_ymin; j < spprs_ymax; j = j + 1)
			{
				if (grayImg.at<uchar>(j,i)<graysupp)
				{

				}
				else
					binaryimg.at<uchar>(j, i) = 255;

			}

		// imshow("binary", binaryimg);
		// waitKey(0);

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
				else angle = abs(boundRect[i].angle);
			}
		}
		// imshow("result", copyimg);
		// waitKey(0);
	}



	vector<float> single(Mat& depImg, Mat& colorImg)
	{

        float angle = 0;
        Point2d center = Point2d(0, 0);
        singleobject(colorImg, depImg, angle, center);

        float d = depImg.at<float>(int(center.y), int(center.x));
        
        if (d == 0)
        {
            int temp_x = int(center.x)-40, temp_y= int(center.y)-40;
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

        vector<float>p(4);//012--xyz
        
        p[2] = float(d) / camera_factor;
        p[0] = (center.x - camera_cx) * p[2] / camera_fx;
        p[1] = (center.y - camera_cy) * p[2] / camera_fy;
		p[3] = angle;
		
		cout<<"result:"<<p[0]<<","<<p[1]<<","<<p[2]<<","<<p[3]<<endl;
		return p;
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_process");
	ros::NodeHandle nh;

	ImgProc imageProcess(nh);
	
	while(ros::ok()){
		ros::spin();
	}
}

