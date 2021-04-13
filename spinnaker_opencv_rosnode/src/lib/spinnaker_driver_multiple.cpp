#include "spinnaker_driver_multiple.h"

SystemPtr cam_system;
CameraList camList;


using std::string;
int PrintDeviceInfo(INodeMap & nodeMap,std::string& output_dev_id)
{
    int result = 0;

    std::cout << std::endl << "*** DEVICE INFORMATION ***" << std::endl << std::endl;
    try
    {
        FeatureList_t features;
        CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);
            FeatureList_t::const_iterator it;
            for (it = features.begin(); it != features.end(); ++it)
            {
                CNodePtr pfeatureNode = *it;
                std::cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = (CValuePtr)pfeatureNode;
                std::stringstream ss;
                ss<<pfeatureNode->GetName();
                // if(ss.str() == "DeviceID")
                if(ss.str() == "DeviceSerialNumber")
                {
                    std::stringstream output_id_ss;
                    output_id_ss<<pValue->ToString();
                    output_dev_id = output_id_ss.str();
                }
                std::cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                std::cout << std::endl;
            }
        }
        else
        {
            std::cout << "Device control information not available." << std::endl;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        result = -1;
    }

    return result;
}

void setEnumValue(CameraPtr pCam_,string setting, string value) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr))
        std::cout<<"Unable to set " << setting << " to " << value << " (enum retrieval). Aborting..."<<std::endl;

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrValue = ptr->GetEntryByName(value.c_str());
    if (!IsAvailable(ptrValue) || !IsReadable(ptrValue))
        std::cout<<"Unable to set " << setting << " to " << value << " (entry retrieval). Aborting..."<<std::endl;
		
    // retrieve value from entry node
    int64_t valueToSet = ptrValue->GetValue();
		
    // Set value from entry node as new value of enumeration node
    ptr->SetIntValue(valueToSet);    

    std::cout<<setting << " set to " << value<<std::endl;
    
}

void setIntValue(CameraPtr pCam_,string setting, int val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CIntegerPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        std::cout<<"Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting..."<<std::endl;
    }
    ptr->SetValue(val);

    std::cout<<setting << " set to " << val<<std::endl;
    
}

void setFloatValue(CameraPtr pCam_,string setting, float val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CFloatPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        std::cout<<"Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting..."<<std::endl;
    }
    ptr->SetValue(val);

    std::cout<<setting << " set to " << val<<std::endl;
    
}

void setBoolValue(CameraPtr pCam_,string setting, bool val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CBooleanPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        std::cout<<"Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting..."<<std::endl;
    }
    if (val) ptr->SetValue("True");
		else ptr->SetValue("False");

    std::cout<<setting << " set to " << val<<std::endl;
    
}


int SpinnakerDriverMultiple::initAllCams()
{
    int result = 0;

    // Print application build information
    //std::cout << "Application build date: " << __DATE__ << " " << __TIME__ << std::endl << std::endl;

    // Retrieve singleton reference to system object
    cam_system = System::GetInstance();
    // Retrieve list of cameras from the system
    camList = cam_system->GetCameras();
    unsigned int numCameras = camList.GetSize();

    std::cout << "Number of cameras detected: " << numCameras << std::endl << std::endl;

    for(int i = 0;i<numCameras;i++)
    {
        CameraPtr pCam = camList.GetByIndex(i);
        //std::stringstream ssInstanceID;
        std::string cam_id;
        spinnaker_driver_single cam;
        cam.pCam = pCam;
        try
        {
            // Retrieve TL device nodemap and print device information
            INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

            result = PrintDeviceInfo(nodeMapTLDevice,cam_id);
            // Initialize camera
            pCam->Init();
        }
        catch (Spinnaker::Exception &e)
        {
            std::cout << "Error occured while initializing cam "<<i<<": " << e.what() << std::endl;

            return -1;
        }
        INodeMap & nodeMap = pCam->GetNodeMap();
        this->cams[cam_id] =cam;
        CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
        std::cout << "Pixel format is " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << std::endl;
        printf("isavailable: %d, iswritable: %d\n", IsAvailable(ptrPixelFormat), IsWritable(ptrPixelFormat));
        if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat))
        {
            // Retrieve the desired entry node from the enumeration node
            CEnumEntryPtr ptrPixelFormatRGB8 = ptrPixelFormat->GetEntryByName("BGR8");
            printf("isavailable: %d, isreadable: %d\n", IsAvailable(ptrPixelFormatRGB8), IsReadable(ptrPixelFormatRGB8));
            if (IsAvailable(ptrPixelFormatRGB8) && IsReadable(ptrPixelFormatRGB8))
            {
                // Retrieve the integer value from the entry node
                int64_t pixelFormatRGB8 = ptrPixelFormatRGB8->GetValue();
                // Set integer as new value for enumeration node
                ptrPixelFormat->SetIntValue(pixelFormatRGB8);
                std::cout << "Pixel format set to " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << std::endl;
            }
            else
            {
                std::cout << "Pixel format not available..." << std::endl;
                std::cout << "Pixel format is " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << std::endl;
            }
        }
        else
        {
            std::cout << "Pixel format not available..." << std::endl;
        }

        //---Change rgb transform mode.
        setEnumValue(pCam,"RgbTransformLightSource","CoolFluorescent4000K");
//        //---Change rgb transform mode.
        setEnumValue(pCam,"GainAuto","Continuous");
        setEnumValue(pCam,"ExposureAuto","Continuous");
        setEnumValue(pCam,"AcquisitionMode","Continuous");
        const float target_grey_value_lower = 3.93f;
        const float target_grey_value_upper = 7.0f;
        //const float target_grey_value_upper = 24.0f;
        setFloatValue(pCam,"AutoExposureGreyValueLowerLimit", target_grey_value_lower);
        setFloatValue(pCam,"AutoExposureGreyValueUpperLimit", target_grey_value_upper);

        const float gamma_value = 0.449;
        setFloatValue(pCam,"Gamma",gamma_value);

        const float exposure_time_min_us = 100;
        const float exposure_time_max_us = 2500;
        setFloatValue(pCam,"AutoExposureExposureTimeLowerLimit",exposure_time_min_us);
        setFloatValue(pCam,"AutoExposureExposureTimeUpperLimit",exposure_time_max_us);

        const float gain_min_value_db = 0;
        //const float gain_max_value_db = 36; //64 times.
        //const float gain_max_value_db = 30.1; // 32 times.
        const float gain_max_value_db = 27.6; //24 times.
        setFloatValue(pCam,"AutoExposureGainLowerLimit",gain_min_value_db);
        setFloatValue(pCam,"AutoExposureGainUpperLimit",gain_max_value_db);

        setEnumValue(pCam,"TriggerMode","Off");
        setEnumValue(pCam,"TriggerSelector","FrameStart"); // need test for multiple cams.
        //setEnumValue(pCam,"TriggerSelector","AcquisitionStart");
        setEnumValue(pCam,"TriggerSource","Software");
        setEnumValue(pCam,"TriggerMode","On");
        //setBoolValue(pCam,"AcquisitionFrameRateEnable",true);

        //setFloatValue(pCam,"AcquisitionFrameRate",9.0);

        pCam->BeginAcquisition();
        std::cout << "Acquiring images..." << std::endl;

        gcstring deviceSerialNumber("");
        CStringPtr ptrStringSerial = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            deviceSerialNumber = ptrStringSerial->GetValue();
            std::cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << std::endl;
        }
        std::cout <<""<< std::endl;
    }
    //result = 0;
    return result;
}
int SpinnakerDriverMultiple::triggerAllCams()
{
    //TODO:
    int success_count = 0;
    for(auto iter=this->cams.begin();iter!=this->cams.end();++iter)
    {
        INodeMap & nodeMap = iter->second.pCam->GetNodeMap();
        CCommandPtr ptrSoftwareTriggerCommand = nodeMap.GetNode("TriggerSoftware");
        if (!IsAvailable(ptrSoftwareTriggerCommand) || !IsWritable(ptrSoftwareTriggerCommand))
        {
            std::cout << "[FLIR_CAMERA_DRIVER] [ERROR] Unable to execute trigger for cam id:" << iter->first<<"; You may get old images!"<< std::endl;
            continue;
        }
        ptrSoftwareTriggerCommand->Execute();
        success_count++;
    }
    return success_count;
}
int SpinnakerDriverMultiple::getFrameByID(std::string cam_id, cv::Mat &frame)
{

    //int AcquireImage(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
    int result = 0;
    try
    {
        auto cam =  this->cams.at(cam_id);
        auto pCam = cam.pCam;
        auto& pResultImage = cam.pResultImage;
        pResultImage = pCam->GetNextImage();
        if (pResultImage->IsIncomplete())
        {
            std::cout << "[FLIR_CAMERA_DRIVER] [WARNING]Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << std::endl << std::endl;
            result = -1;
        }
        else
        {
            //size_t width = pResultImage->GetWidth();
            //size_t height = pResultImage->GetHeight();
            //std::cout << "Grabbed image, width = " << width << ", height = " << height << ", format = " << (pCam->PixelFormat.GetValue() == PixelFormat_BGR8) << std::endl;
            cv::Mat frame_original = cv::Mat(pResultImage->GetHeight() + pResultImage->GetYPadding(), pResultImage->GetWidth() + pResultImage->GetXPadding(), CV_8UC3);
            if (cam.firstGetFrame)
            {
                frame =  cv::Mat(pResultImage->GetHeight() + pResultImage->GetYPadding(), pResultImage->GetWidth() + pResultImage->GetXPadding(), CV_8UC3);
                cam.firstGetFrame = false;
            }
            frame.data = (uchar*)pResultImage->GetData();
        }
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        result = -1;
    }
    return result;
}
int SpinnakerDriverMultiple::releaseFrameByID(std::string cam_id)
{
    int result = 0;
    try
    {
        this->cams.at(cam_id).pResultImage->Release();
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        result = -1;
    }
    return result;
}
void SpinnakerDriverMultiple::shutdownAll()
{
    for(auto iter = this->cams.begin();iter!=this->cams.end();++iter)
    {
        //
        // End acquisition
        //
        // *** NOTES ***
        // Ending acquisition appropriately helps ensure that devices clean up
        // properly and do not need to be power-cycled to maintain integrity.
        //
        auto pCam = iter->second.pCam;
        pCam->EndAcquisition();

        // Deinitialize camera
        pCam->DeInit();
        //
        // Release reference to the camera
        //
        // *** NOTES ***
        // Had the CameraPtr object been created within the for-loop, it would not
        // be necessary to manually break the reference because the shared pointer
        // would have automatically cleaned itself up upon exiting the loop.
        //
        pCam = 0;
    }
    // Clear camera list before releasing system
    cams.erase(cams.begin(),cams.end());
    camList.Clear();
    // Release system
    cam_system->ReleaseInstance();
    //std::cout << std::endl << "Done! Press Enter to exit..." << std::endl;
    //getchar();

}
