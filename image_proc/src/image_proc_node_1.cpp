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
 
#define DEBUG_MODE
using namespace std;
using namespace cv;
 
enum
{
	Processor_cl,
	Processor_gl,
	Processor_cpu
};
 
void sigint_handler(int s)
{
	protonect_shutdown = true;
}
 
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
        server_ = nh_.advertiseService("Image_Process_A", &imageProcessCallback);

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
        signal(SIGINT, sigint_handler);
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
		// cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

        listener->release(frames);

		
		image_proc_msgs::ImageProcMsg temp[1];
		vector<float> pos = {0,0,0};
		temp[0].position = pos;
		resp.object = temp;
		ROS_INFO("ImageProcess handle success!");
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
