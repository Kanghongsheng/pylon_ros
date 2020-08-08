//
// Created by linux on 2020/1/2.
//
#include <iostream>
#include <pylon/PylonIncludes.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


static const uint32_t c_countOfImagesToGrab = 100;

int main(int argc, char* argv[]){
    ros::init(argc, argv, "pylon_node_1" );
    ros::NodeHandle pylon;
    ros::Publisher image_pub = pylon.advertise<sensor_msgs::Image>("/pylon/raw_image_1", 10);

    int exitCode = 0;
    int count = 0;
    Pylon::PylonInitialize();

    Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();
    cv::namedWindow("pylon_image_1",0);
    //cv::resizeWindow("pylon_image_1", 640, 480);

    try {

         Pylon::CDeviceInfo a_camera;
         a_camera.SetSerialNumber("21906918");
	//net: 21159245 usb: 21906918
         //a_camera.SetIpAddress(argv[1]);
        Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateDevice(a_camera));
	//Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        std::cout << "Using device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        std::cout << "SerialNumber: " << camera.GetDeviceInfo().GetSerialNumber() << std::endl;
        camera.MaxNumBuffer = 5;
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        Pylon::CGrabResultPtr ptrGrabResult;
        Pylon::CPylonImage target;
        while (camera.IsGrabbing()) {
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            std_msgs::Header header;
            header.frame_id = "Pylon_camera_1";
            header.stamp    = ros::Time::now();

            if (ptrGrabResult->GrabSucceeded()) {
                Pylon::CImageFormatConverter converter;
                converter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
                converter.OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;
                converter.Convert(target, ptrGrabResult);
		//std::cout << target.GetHeight() << ", " << target.GetWidth() << std::endl;
                cv::Mat Image(target.GetHeight(), target.GetWidth(), CV_8UC3, target.GetBuffer(), cv::Mat::AUTO_STEP);
		cv::Mat mini_Image;
		cv::resize(Image, mini_Image, cv::Size(960, 600));
                sensor_msgs::ImagePtr imageMsg(new sensor_msgs::Image);
                imageMsg = cv_bridge::CvImage(header, "bgr8", mini_Image).toImageMsg();
                image_pub.publish(*imageMsg);
                ptrGrabResult.Release();
                cv::imshow("pylon_image_1", mini_Image);
                if(cv::waitKey(1)==27)break;
                std::cout << ++count << " : " << header.stamp << std::endl;
            } else {
                std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription()
                          << std::endl;
            }
        }
    }
    catch (const Pylon::GenericException &e)
    {
        std::cerr << "An exception occurred." << std::endl << e.GetDescription() << std::endl;
        exitCode = 1;
    }


    Pylon::PylonTerminate();
    return 0;
}

