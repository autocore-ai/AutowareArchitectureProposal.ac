#ifndef MULTI_CAM_NODE_H_
#define MULTI_CAM_NODE_H_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <string>
#include <vector>
#include <map>
using namespace std;

class ImagePuber:public rclcpp::Node
{
public:
    ImagePuber();
    ~ImagePuber();

    void remove_distorition(const cv::Mat& frame,cv::Mat& remapped_frame,string cam_id);

    void init_puber(vector<string> ids);

    void read_camera_param();

    void pub_image(string cam_id,const cv::Mat& frame);
private:
    struct CameraParam
    {
        cv::Mat old_k;
        cv::Mat old_d;
        cv::Mat k;

        string topic_name; 

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr puber;
        cv::Mat map1;
        cv::Mat map2;
    };

    map<string,CameraParam> m_id2param_;
};

#endif