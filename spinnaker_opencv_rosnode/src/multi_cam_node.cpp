#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <csignal>
#include <cstdio>
#include <iostream>

#include "utils.h"
#include "spinnaker_driver_multiple.h"

// #include <ros/ros.h>
#include <unistd.h>
// #include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/persistence.hpp"
#include <Timer.h>
#include "multi_cam_node.h"

ImagePuber::ImagePuber():Node("image_puber")
{

}

ImagePuber::~ImagePuber()
{

}

void ImagePuber::read_camera_param()
{
    cout<<"read begin"<<endl;

    m_id2param_.clear();
    cv::FileStorage fs;
    string pkg_path = ament_index_cpp::get_package_share_directory("spinnaker_opencv_rosnode");
    string calib_file_path = pkg_path+"/launch/Calibs.yaml";
    fs.open(calib_file_path,cv::FileStorage::READ);
    cout<<calib_file_path<<endl;
    cv::FileNode root = fs.root();
    for(auto it = root.begin();it!=root.end();it++)
    {
        cv::FileNode item = *it;
        string cam_id = item.name().substr(3);
        if(!item.isMap()&&item.keys()[0]!=string("CamOldK"))
        {
            continue;
        }

        // cout<<"........."<<endl;
        struct CameraParam camera_param;

        item["CamOldK"]>>camera_param.old_k;
        item["CamOldD"]>>camera_param.old_d;
        item["CameraMat"]>>camera_param.k;
        camera_param.topic_name = item["topic_name"].string();

        cv::initUndistortRectifyMap(camera_param.old_k,camera_param.old_d,cv::Mat(),
            camera_param.k,cv::Size(1440,1080),CV_32F,camera_param.map1,camera_param.map2);

        m_id2param_.insert(make_pair(cam_id,camera_param));
    }

    cout<<"read end"<<endl;

#if 1
    cout<<""<<endl;
    for(auto &v : m_id2param_)
    {
        cout<<v.first<<endl;
        cout<<v.second.old_k<<endl;
        cout<<v.second.old_d<<endl;
        cout<<v.second.k<<endl;
    }
#endif
    fs.release();
}

void ImagePuber::init_puber(vector<string> ids)
{
    for(string id : ids)
    {
        if(m_id2param_.find(id) == m_id2param_.end())
        {
            cout<<"cam"<<id<<" param not found"<<endl;
            continue;
        }

        CameraParam & param = m_id2param_[id];
        string & topic = param.topic_name;
        param.puber  = this->create_publisher<sensor_msgs::msg::Image>(topic, 1);
    }
}

void ImagePuber::remove_distorition(const cv::Mat& frame,cv::Mat& remapped_frame,string cam_id)
{
    if(m_id2param_.find(cam_id) != m_id2param_.end())
    {
        CameraParam & param = m_id2param_[cam_id];

        cv::remap(frame,remapped_frame,param.map1,param.map2,cv::INTER_LINEAR);  
    }
}

void ImagePuber::pub_image(string cam_id,const cv::Mat& frame)
{
    if(m_id2param_.find(cam_id) != m_id2param_.end())
    {
        CameraParam & param = m_id2param_[cam_id];
        
        auto header = std_msgs::msg::Header();
        auto time_stamp = rclcpp::Clock().now();
	    header.stamp = time_stamp;
        header.frame_id = "test";
        auto imgPtr = std::make_shared<cv_bridge::CvImage> (header,"bgr8",frame);
        auto msg = *(imgPtr->toImageMsg());

        auto & puber = param.puber;
        puber->publish(msg);
    }
}

int main(int argc,char** argv)
{
    rclcpp::init(argc, argv);

    shared_ptr<ImagePuber> pub_node = make_shared<ImagePuber>();
    pub_node->read_camera_param();

    SpinnakerDriverMultiple sdm;
    int err = sdm.initAllCams();
    if(err)
    {
        cout<<"error in initAllCams."<<endl;
        return -1;
    }

    auto ids = sdm.getIDs();
    for(auto & id : ids)
    {
        cout<<"cam id:"<<id<<endl;
    }
    pub_node->init_puber(ids);

    while(rclcpp::ok())
    {
        sdm.triggerAllCams();
        cv::Mat frame;
        cv::Mat remapped_frame;
        for(string& id:ids)
        {
            if(-1 == sdm.getFrameByID(id,frame))
            {
                cout<<"error in get frame.cam id="<<id<<endl;
                continue;
            }

            pub_node->remove_distorition(frame,remapped_frame,id);
            pub_node->pub_image(id,remapped_frame);
        }
    }

    rclcpp::spin(pub_node);
    rclcpp::shutdown();

    return 0;
}
