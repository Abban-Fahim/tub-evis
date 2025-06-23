#pragma once

#include <memory.h>
#include <string.h>
#include <deque>
#include <glog/logging.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

// Replaced kindr with a diff library for transformations 
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dvs_mosaic
{

// using Transformation = kindr::minimal::QuatTransformation;
//using Transformation = kindr::minimal::RotationQuaternion;
using Transformation = Eigen::Affine3d;

class Mosaic : public rclcpp::Node {
  public:
    Mosaic();
    virtual ~Mosaic();

  private:
    // Private handles for ROS functionality
    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport it_;

    // Callback functions
    void eventsCallback(const dvs_msgs::msg::EventArray::ConstPtr& msg);

    // Subscribers
    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_sub_;

    // Publishers
    image_transport::Publisher mosaic_pub_;
    image_transport::Publisher time_map_pub_;
    image_transport::Publisher mosaic_gradx_pub_, mosaic_grady_pub_, mosaic_tracecov_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    void publishMap();
    void publishPose();
    rclcpp::Time time_packet_;

    // Sliding window of events
    std::deque<dvs_msgs::msg::Event> events_;
    std::vector<dvs_msgs::msg::Event> events_subset_;

    // Camera
    int sensor_width_, sensor_height_;
    image_geometry::PinholeCameraModel dvs_cam_;
    cv::Mat time_map_;

    // Mosaic parameters
    int mosaic_width_, mosaic_height_;
    cv::Size mosaic_size_;
    float fx_, fy_; // speed-up equiareal projection

    // Measurement function
    bool measure_contrast_;
    float var_R_;
    float C_th_;

    // Mapping / mosaicing
    int num_events_map_update_;
    int idx_first_ev_map_;  // index of first event of processing window
    std::vector<cv::Matx33d> map_of_last_rotations_;
    cv::Mat grad_map_, grad_map_covar_, mosaic_img_;
    std::map<rclcpp::Time, Transformation> poses_;
    void loadPoses();
    void processEventForMap(const dvs_msgs::msg::Event& ev, const double t_ev,
      const double t_prev, const cv::Matx33d& Rot, const cv::Matx33d& Rot_prev);
    bool rotationAt(const rclcpp::Time& t_query, cv::Matx33d& Rot_interp);
    void project_EquirectangularProjection(const cv::Point3d& pt_3d, cv::Point2f& pt_on_mosaic);

    // Precomputed bearing vectors for each camera pixel
    std::vector<cv::Point3d> precomputed_bearing_vectors_;
    void precomputeBearingVectors();
};

} // namespace
