#include <dvs_mosaic/mosaic.h>

namespace dvs_mosaic {

Mosaic::Mosaic() : rclcpp::Node("integrator_conv"), it_(shared_from_this()) {

  // Declate and fetch parameters
  declare_parameter("num_events_map_update", 10000);
  declare_parameter("mosaic_height", 1024);
  declare_parameter("variance_init_grad", 10.);
  num_events_map_update_ = this->get_parameter("num_events_map_update").as_int();
  mosaic_height_ = this->get_parameter("mosaic_height").as_int();
  float grad_init_variance = this->get_parameter("variance_init_grad").as_double();
  
  // Create event subscription
  event_sub_ = create_subscription<dvs_msgs::msg::EventArray>("/dvs/events", 10, Mosaic::eventsCallback);

  // Create publishers
  mosaic_pub_ = it_.advertise("/mosaic_img", 10);
  time_map_pub_ = it_.advertise("/time_map", 10);
  mosaic_gradx_pub_ = it_.advertise("/mosaic_gradx", 10);
  mosaic_grady_pub_ = it_.advertise("/mosaic_grady", 10);
  mosaic_tracecov_pub_ = it_.advertise("/mosaic_tracecov", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/cam_pose", 10);

  // Batch processing of events
  idx_first_ev_map_ = 0;
  time_packet_ = rclcpp::Time(0);

}

void Mosaic::eventsCallback(const dvs_msgs::msg::EventArray::ConstPtr& msg) {
  // Append events to queue like previous excercise
  for(const dvs_msgs::msg::Event& ev : msg->events)
    events_.push_back(ev);

  static unsigned int packet_number = 0;
  static unsigned long total_event_count = 0;
  total_event_count += msg->events.size();
  VLOG(1) << "Packet # " << packet_number << "  event# " << total_event_count << "  queue_size:" << events_.size();

  // Initialize time map (negative indicates first init)
  if (packet_number == 0) {
    time_map_ = cv::Mat(sensor_width_, sensor_height_, CV_64FC1, cv::Scalar(-0.01));
  }
  packet_number++;

  

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mosaic>());
  rclcpp::shutdown();
  return 0;
}

};