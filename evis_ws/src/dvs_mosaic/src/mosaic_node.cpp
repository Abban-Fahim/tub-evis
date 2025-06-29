#include <dvs_mosaic/mosaic.h>

namespace dvs_mosaic {

void Mosaic::init() {
  it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
  mosaic_pub_ = it_->advertise("/mosaic_img", 10);
  time_map_pub_ = it_->advertise("/time_map", 10);
  mosaic_gradx_pub_ = it_->advertise("/mosaic_gradx", 10);
  mosaic_grady_pub_ = it_->advertise("/mosaic_grady", 10);
  mosaic_tracecov_pub_ = it_->advertise("/mosaic_trace_cov", 10);
}

Mosaic::Mosaic() : rclcpp::Node("integrator_conv") {
  // Declate and fetch parameters
  declare_parameter("num_events_map_update", 10000);
  declare_parameter("mosaic_height", 1024);
  declare_parameter("variance_init_grad", 10.);
  num_events_map_update_ = this->get_parameter("num_events_map_update").as_int();
  mosaic_height_ = this->get_parameter("mosaic_height").as_int();
  float grad_init_variance = this->get_parameter("variance_init_grad").as_double();
  
  // Create event subscription
  event_sub_ = create_subscription<dvs_msgs::msg::EventArray>("/dvs/events", 10, std::bind(&Mosaic::eventsCallback, this, std::placeholders::_1));

  // Create publisher
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mosaic_pose", 10);

  // Batch processing of events
  idx_first_ev_map_ = 0;
  time_packet_ = rclcpp::Time(0);

  // Setup camera and precompute bearing vectors
  camera_info_manager::CameraInfoManager camInfo(this, "DVS-syntetic", ament_index_cpp::get_package_share_directory("dvs_mosaic") + "/data/DVS-synthetic.yaml");
  dvs_cam_.fromCameraInfo(camInfo.getCameraInfo());
  sensor_width_ = dvs_cam_.fullResolution().width;
  sensor_height_ = dvs_cam_.fullResolution().height;
  precomputeBearingVectors();

  // Setup mosaic info
  mosaic_width_ = 2 * mosaic_height_;
  mosaic_size_ = cv::Size(mosaic_width_, mosaic_height_);
  fx_ = 0.5 * M_1_PI * mosaic_width_;
  fy_ = M_1_PI * mosaic_height_;

  // Measurement function
  C_th_ = 0.45;
  measure_contrast_ = false;
  if (measure_contrast_)
    var_R_ = 0.17 * 0.17;
  else
    var_R_ = 1e4;

  // Mapping variables
  map_of_last_rotations_.resize(mosaic_size_.area());
  grad_map_ = cv::Mat::zeros(mosaic_size_, CV_32FC2);
  grad_map_covar_ = cv::Mat(mosaic_size_, CV_32FC2, cv::Scalar(grad_init_variance, 0.f, grad_init_variance));
  
  // Get ground truth poses from file
  poses_.clear();
  loadPoses();
}

void Mosaic::eventsCallback(const dvs_msgs::msg::EventArray::ConstPtr& msg) {
  // Append events to queue like previous excercise
  for(const dvs_msgs::msg::Event& ev : msg->events)
    events_.push_back(ev);

  static unsigned int packet_number = 0;
  static unsigned long total_event_count = 0;
  total_event_count += msg->events.size();
  RCLCPP_DEBUG(get_logger(), "Packet # %d  event# %d  queue_size: %lu", 
    packet_number, total_event_count, events_.size());

  // Initialize time map (negative indicates first init)
  if (packet_number == 0) {
    time_map_ = cv::Mat(sensor_width_, sensor_height_, CV_64FC1, cv::Scalar(-0.01));
  }
  packet_number++;

  // Mapping algorithm
  while (idx_first_ev_map_ + num_events_map_update_ <= events_.size()) {
    // Get subset of events
    events_subset_ = std::vector<dvs_msgs::msg::Event>(events_.begin() + idx_first_ev_map_,
                                                   events_.begin() + idx_first_ev_map_ + num_events_map_update_);

    // Get midpoint of event times for publishing
    rclcpp::Time first_time(events_subset_.front().ts);
    rclcpp::Time last_time(events_subset_.back().ts);
    rclcpp::Duration d_time = last_time - first_time;
    time_packet_ = first_time + rclcpp::Duration::from_nanoseconds(d_time.nanoseconds() * 0.5);
    
    // Call the mapping functions
    
    
    // Call the integration algorithm in batches of 300
    const int num_events_share_Rot = 300;
    int idx_first_ev_batch = 0;
    while (idx_first_ev_batch < events_subset_.size()) {
      // Make an event batch out of the subset
      int num_ev_batch = std::min(num_events_share_Rot, int(events_subset_.size() - idx_first_ev_batch));
      std::vector<dvs_msgs::msg::Event> events_batch =
      std::vector<dvs_msgs::msg::Event>(events_subset_.begin() + idx_first_ev_batch,
      events_subset_.begin() + idx_first_ev_batch + num_ev_batch);
      
      // Find midpoint time in batch, and find camera rotation at that time
      rclcpp::Time first_t(events_batch.front().ts);
      rclcpp::Time last_t(events_batch.back().ts);
      rclcpp::Duration dt = last_t - first_t;
      rclcpp::Time batch_time = first_t + rclcpp::Duration::from_nanoseconds(dt.nanoseconds() * 0.5);
      cv::Matx33d rot;
      rotationAt(batch_time, rot);

      for (const dvs_msgs::msg::Event& ev : events_batch) {
        const double t_ev = ev.ts.sec + ev.ts.nanosec * 1e-9;
        double t_prev = time_map_.at<double>(cv::Point(ev.x, ev.y));
        if (t_prev == -0.01)
          RCLCPP_DEBUG(get_logger(), "got -1");
      }
    }

    publishMap();
    idx_first_ev_map_ += num_events_map_update_;

  }

}

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "staring node";
  auto node = std::make_shared<dvs_mosaic::Mosaic>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}