// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lidar_centerpoint/node.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <config.hpp>
#include <pcl_ros/transforms.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <string>
#include <vector>

namespace centerpoint
{
LidarCenterPointNode::LidarCenterPointNode(const rclcpp::NodeOptions & node_options)
: Node("lidar_center_point", node_options)
{
  score_threshold_ = this->declare_parameter("score_threshold", 0.4);
  densification_base_frame_ = this->declare_parameter("densification_base_frame", "map");
  densification_past_frames_ = this->declare_parameter("densification_past_frames", 1);
  use_encoder_trt_ = this->declare_parameter("use_encoder_trt", false);
  use_head_trt_ = this->declare_parameter("use_head_trt", true);
  trt_precision_ = this->declare_parameter("trt_precision", "fp16");
  encoder_onnx_path_ = this->declare_parameter("encoder_onnx_path", "");
  encoder_engine_path_ = this->declare_parameter("encoder_engine_path", "");
  encoder_pt_path_ = this->declare_parameter("encoder_pt_path", "");
  head_onnx_path_ = this->declare_parameter("head_onnx_path", "");
  head_engine_path_ = this->declare_parameter("head_engine_path", "");
  head_pt_path_ = this->declare_parameter("head_pt_path", "");
  class_names_ = this->declare_parameter<std::vector<std::string>>("class_names");
  rename_car_to_truck_and_bus_ = this->declare_parameter("rename_car_to_truck_and_bus", false);

  NetworkParam encoder_param(
    encoder_onnx_path_, encoder_engine_path_, encoder_pt_path_, trt_precision_, use_encoder_trt_);
  NetworkParam head_param(
    head_onnx_path_, head_engine_path_, head_pt_path_, trt_precision_, use_head_trt_);
  densification_ptr_ = std::make_unique<PointCloudDensification>(
    densification_base_frame_, densification_past_frames_, this->get_clock());
  detector_ptr_ = std::make_unique<CenterPointTRT>(encoder_param, head_param, /*verbose=*/false);

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&LidarCenterPointNode::pointCloudCallback, this, std::placeholders::_1));
  objects_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
      "~/output/objects", rclcpp::QoS{1});
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/debug/pointcloud_densification", rclcpp::SensorDataQoS{}.keep_last(1));
}

void LidarCenterPointNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg)
{
  const auto objects_sub_count =
    objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
  const auto pointcloud_sub_count = pointcloud_pub_->get_subscription_count() +
                                    pointcloud_pub_->get_intra_process_subscription_count();
  if (objects_sub_count < 1 && pointcloud_sub_count < 1) {
    return;
  }

  auto stacked_pointcloud_msg = densification_ptr_->stackPointCloud(*input_pointcloud_msg);
  std::vector<float> boxes3d_vec = detector_ptr_->detect(stacked_pointcloud_msg);

  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray output_msg;
  output_msg.header = input_pointcloud_msg->header;
  for (size_t obj_i = 0; obj_i < boxes3d_vec.size() / Config::num_box_features; obj_i++) {
    float score = boxes3d_vec[obj_i * Config::num_box_features + 0];
    if (score < score_threshold_) {
      continue;
    }

    int class_id = static_cast<int>(boxes3d_vec[obj_i * Config::num_box_features + 1]);
    float x = boxes3d_vec[obj_i * Config::num_box_features + 2];
    float y = boxes3d_vec[obj_i * Config::num_box_features + 3];
    float z = boxes3d_vec[obj_i * Config::num_box_features + 4];
    float w = boxes3d_vec[obj_i * Config::num_box_features + 5];
    float l = boxes3d_vec[obj_i * Config::num_box_features + 6];
    float h = boxes3d_vec[obj_i * Config::num_box_features + 7];
    float yaw = boxes3d_vec[obj_i * Config::num_box_features + 8];
    float vel_x = boxes3d_vec[obj_i * Config::num_box_features + 9];
    float vel_y = boxes3d_vec[obj_i * Config::num_box_features + 10];

    autoware_perception_msgs::msg::DynamicObjectWithFeature feature_obj;
    feature_obj.object.semantic.type = getSemanticType(class_names_[class_id]);
    feature_obj.object.semantic.confidence = score;

    geometry_msgs::msg::Pose pose;
    pose.position = autoware_utils::createPoint(x, y, z);
    pose.orientation = autoware_utils::createQuaternionFromYaw(yaw);
    feature_obj.object.state.pose_covariance.pose = pose;

    autoware_perception_msgs::msg::Shape shape;
    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    shape.dimensions.x = l;
    shape.dimensions.y = w;
    shape.dimensions.z = h;
    feature_obj.object.shape = shape;

    if (
      rename_car_to_truck_and_bus_ &&
      feature_obj.object.semantic.type == autoware_perception_msgs::msg::Semantic::CAR) {
      // Note: object size is referred from multi_object_tracker
      if ((w * l > 2.2 * 5.5) && (w * l <= 2.5 * 7.9)) {
        feature_obj.object.semantic.type = autoware_perception_msgs::msg::Semantic::TRUCK;
      } else if (w * l > 2.5 * 7.9) {
        feature_obj.object.semantic.type = autoware_perception_msgs::msg::Semantic::BUS;
      }
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = std::sqrt(std::pow(vel_x, 2) + std::pow(vel_y, 2));
    twist.angular.z = 2 * (std::atan2(vel_y, vel_x) - yaw);
    feature_obj.object.state.twist_covariance.twist = twist;
    feature_obj.object.state.twist_reliable = true;

    output_msg.feature_objects.emplace_back(feature_obj);
  }

  if (objects_sub_count > 0) {
    objects_pub_->publish(output_msg);
  }
  if (pointcloud_sub_count > 0) {
    pointcloud_pub_->publish(stacked_pointcloud_msg);
  }
}

uint8_t LidarCenterPointNode::getSemanticType(const std::string & class_name)
{
  if (class_name == "CAR") {
    return autoware_perception_msgs::msg::Semantic::CAR;
  } else if (class_name == "TRUCK") {
    return autoware_perception_msgs::msg::Semantic::TRUCK;
  } else if (class_name == "BUS") {
    return autoware_perception_msgs::msg::Semantic::BUS;
  } else if (class_name == "BICYCLE") {
    return autoware_perception_msgs::msg::Semantic::BICYCLE;
  } else if (class_name == "MOTORBIKE") {
    return autoware_perception_msgs::msg::Semantic::MOTORBIKE;
  } else if (class_name == "PEDESTRIAN") {
    return autoware_perception_msgs::msg::Semantic::PEDESTRIAN;
  } else if (class_name == "ANIMAL") {
    return autoware_perception_msgs::msg::Semantic::ANIMAL;
  } else {
    return autoware_perception_msgs::msg::Semantic::UNKNOWN;
  }
}

}  // namespace centerpoint

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(centerpoint::LidarCenterPointNode)
