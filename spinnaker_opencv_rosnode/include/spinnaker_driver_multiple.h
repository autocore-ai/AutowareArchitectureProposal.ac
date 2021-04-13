#ifndef SPINAKER_MULTIPLE_DRIVER
#define SPINAKER_MULTIPLE_DRIVER
#include <string>
#include <sstream>
#include <map>
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
using std::map;
using std::vector;
int PrintDeviceInfo(INodeMap & nodeMap);

struct spinnaker_driver_single{
    bool firstGetFrame = true;
    CameraPtr pCam = 0;
    ImagePtr pResultImage;
};


class SpinnakerDriverMultiple
{
private:
    map<std::string,spinnaker_driver_single> cams;
public:

    int initAllCams();
    vector<std::string> getIDs()
    {
        vector<std::string> ids;
        for(auto iter = this->cams.begin();iter!=this->cams.end();++iter)
        {
            ids.push_back(iter->first);
        }
        return ids;
    }
    int getFrameByID(std::string cam_id,cv::Mat& frame);
    int releaseFrameByID(std::string cam_id);
    int triggerAllCams();
    void shutdownAll();
};

#endif
