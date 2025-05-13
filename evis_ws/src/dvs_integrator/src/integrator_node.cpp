#include <dvs_integrator/integrator.h>

class Integrator : public rclcpp::Node {
  public:
    Integrator() : rclcpp::Node("integrator"), node_handle_(std::shared_ptr<Integrator>(this, [](auto *) {})), it_(node_handle_) {

      // this->declare_parameter("mode", "gray");
      this->declare_parameter("filter_cutoff", 5.);
      this->declare_parameter("beta", 1.);
      
      auto event_callback = [this](dvs_msgs::msg::EventArray::SharedPtr msg) -> void {
        double cutoff_freq = this->get_parameter("filter_cutoff").as_double();
        double beta = this->get_parameter("beta").as_double();
        
        if (cv_img_.rows != msg->height && cv_img_.cols != msg->width) {
          RCLCPP_INFO(this->get_logger(), "Initialising images");
          cv_img_ = cv::Mat::zeros(msg->height, msg->width, CV_8U);
          cv_time_img_ = cv::Mat::zeros(msg->height, msg->width, CV_8U);
        }
        
        for (dvs_msgs::msg::Event e: msg->events) {
          event_accum_++;
          if ((event_accum_ % eventsPerFrame_) != 0) {
            double polarity_int = e.polarity ? 1. : -1.;
            RCLCPP_INFO(this->get_logger(), "b: %f %d %d", beta, cv_img_.at<int8_t>(e.y, e.x), polarity_int);
            cv_img_.at<int8_t>(e.y, e.x) = (int) (beta * (double)cv_img_.at<int8_t>(e.y, e.x) + polarity_int);
            RCLCPP_INFO(this->get_logger(), "b: %d", cv_img_.at<int8_t>(e.y, e.x));
          } else {
            cv_img_ = cv::Mat::zeros(msg->height, msg->width, CV_8U);
            this->publishState();
            // abort();
          }
        }
      };
      
      event_sub_ = this->create_subscription<dvs_msgs::msg::EventArray>("/dvs/events", 10, event_callback);
      img_pub_ = it_.advertise("/reconstructed_img", 10);
      time_map_pub_ = it_.advertise("/time_map", 10);
    }
    
    private:
    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport it_;
    
    int event_accum_ = 0;
    int eventsPerFrame_ = 20000; // 2e4
    cv::Mat cv_img_;
    cv::Mat cv_time_img_;

    void publishState() {
      cv_bridge::CvImage img;
      double rmin, rmax;
      minMaxLocRobust(cv_img_, rmin, rmax, 10);
      // RCLCPP_INFO(this->get_logger(), "%f %f", rmin, rmax);
      cv_img_ = (cv_img_ - rmin) / (rmax - rmin) * 255;
      // int range = 255 / (rmax - rmin);
      // cv_img_ = (int) range * (cv_img_ - rmin);
      cv_img_.copyTo(img.image);
      img.encoding = "mono8";
      img_pub_.publish(img.toImageMsg());
    };

    void minMaxLocRobust(const cv::Mat& image, double& rmin, double& rmax, const double& percentage_pixels_to_discard) {
      cv::Mat image_as_row = image.reshape(0,1);
      cv::Mat image_as_row_sorted;
      cv::sort(image_as_row, image_as_row_sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
      image_as_row_sorted.convertTo(image_as_row_sorted, CV_64FC1);
      const int single_row_idx_min = (0.5*percentage_pixels_to_discard/100.)*image.total();
      const int single_row_idx_max = (1 - 0.5*percentage_pixels_to_discard/100.)*image.total();
      rmin = image_as_row_sorted.at<double>(single_row_idx_min);
      rmax = image_as_row_sorted.at<double>(single_row_idx_max);
    }

    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_sub_;
    image_transport::Publisher img_pub_;
    image_transport::Publisher time_map_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Integrator>());
  rclcpp::shutdown();
  return 0;
}
