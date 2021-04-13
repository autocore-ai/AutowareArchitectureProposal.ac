#ifndef FLIR_DRIVER_H_FILE
#define FLIR_DRIVER_H_FILE

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream> 
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "utils.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap & nodeMap);

namespace spinnaker_driver
{


  // Example entry point; please see Enumeration example for more in-depth 
  // comments on preparing and cleaning up the system.
  int init();
  int getFrame(cv::Mat& frame);
//
//  {
//    //int AcquireImage(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
//    int result = 0;
//    
//    try
//    {
//      pResultImage = pCam->GetNextImage();
//      if (pResultImage->IsIncomplete())
//        std::cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << std::endl << std::endl;
//      else
//      {
//        //size_t width = pResultImage->GetWidth();
//        //size_t height = pResultImage->GetHeight();
//  
//        //std::cout << "Grabbed image, width = " << width << ", height = " << height << ", format = " << (pCam->PixelFormat.GetValue() == PixelFormat_BGR8) << std::endl;
//  
//        cv::Mat frame_original = cv::Mat(pResultImage->GetHeight() + pResultImage->GetYPadding(), pResultImage->GetWidth() + pResultImage->GetXPadding(), CV_8UC3);
//        if (firstGetFrame)
//        {
//          printf("firstgetframe\n");
//          
//          frame =  cv::Mat(pResultImage->GetHeight() + pResultImage->GetYPadding(), pResultImage->GetWidth() + pResultImage->GetXPadding(), CV_8UC3);
//
//          firstGetFrame = false;
//        }
//  
//        frame_original.data = (uchar*)pResultImage->GetData();
//        //cv::cvtColor(frame_original,frame,cv::COLOR_RGB2BGR);
//        frame = frame_original;
//  
//        //if (false)
//        //{
//        //  ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
//        //  // Create a unique filename
//        //  std::ostringstream filename;
//  
//        //  filename << "Acquisition-";
//        //  if (deviceSerialNumber != "")
//        //  {
//        //                  filename << deviceSerialNumber.c_str() << "-";
//        //  }
//        //  filename << "file.jpg";
//        //  //
//        //  // Save image
//        //  // 
//        //  // *** NOTES ***
//        //  // The standard practice of the examples is to use device
//        //  // serial numbers to keep images of one device from 
//        //  // overwriting those of another.
//        //  //
//        //  convertedImage->Save(filename.str().c_str());
//        //  std::cout << "Image saved at " << filename.str() << std::endl;
//        //}
//      }
//    }
//    catch (Spinnaker::Exception &e)
//    {
//      std::cout << "Error: " << e.what() << std::endl;
//      result = -1;
//    }
//    
//    return result;
//  }

  int releaseFrame();
  void shutdown();
}
#endif
