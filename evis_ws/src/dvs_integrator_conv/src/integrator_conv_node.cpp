#include <dvs_integrator_conv/integrator_conv.h>

class IntegratorConv : public rclcpp::Node {
  public:
    IntegratorConv() : rclcpp::Node("integrator_conv"), node_handle_(std::shared_ptr<IntegratorConv>(this, [](auto *) {})), it_(node_handle_) {

      this->declare_parameter("filter_cutoff", 5.);
      this->declare_parameter("eventsPerFrame", 20000); // 2e4
      this->declare_parameter("kernel", 0);
      
      auto event_callback = [this](dvs_msgs::msg::EventArray::SharedPtr msg) -> void {
        cutoff_freq_ = this->get_parameter("filter_cutoff").as_double();
        int eventsPerFrame = this->get_parameter("eventsPerFrame").as_int();
        
        if (cv_img_.rows != msg->height && cv_img_.cols != msg->width) {
          RCLCPP_INFO(this->get_logger(), "Initialising images");
          cv_img_ = cv::Mat::zeros(msg->height, msg->width, CV_32F);
          cv_time_img_ = cv::Mat::zeros(msg->height, msg->width, CV_64F);
        }

        int ksize = 3;
        int hksize = ksize / 2;
        switchKernel();
        
        for (dvs_msgs::msg::Event e: msg->events) {
          event_accum_++;
          double curr_time = e.ts.sec + 1e-9 * e.ts.nanosec;

          // Fixed rate of events to publish events
          if ((event_accum_ % eventsPerFrame) != 0) {

            // Check if the event can be convolved or not
            if ((hksize <= e.x) && (e.x <= msg->width-1-hksize) && (hksize <= e.y) && (e.y <= msg->height-1-hksize)) {
              double polarity_int = e.polarity ? 1. : -1.;
              
              cv::Rect rect(e.x - hksize, e.y - hksize, ksize, ksize);
              cv::Mat image_roi(cv_img_, rect);
              cv::Mat time_roi(cv_time_img_, rect);
              
              for (int i = 0; i < ksize; i++) {
                for (int j = 0; j < ksize; j++) {
                  if (conv_kernel_.at<float>(i,j) != 0) {
                    float beta = std::exp(-cutoff_freq_ * (curr_time - time_roi.at<double>(i,j)));
                    image_roi.at<float>(i,j) = beta * image_roi.at<float>(i,j) + polarity_int * conv_kernel_.at<float>(i,j);
                    time_roi.at<double>(i,j) = curr_time;
                  }
                }
              }
            } else {
              cv_time_img_.at<double>(e.y, e.x) = curr_time;
            }
            
            // RCLCPP_INFO(this->get_logger(), "m %f", cv_img_.at<float>(e.y, e.x));
          } else {

            publishState(curr_time);
            cv_img_ = cv::Mat::zeros(msg->height, msg->width, CV_32F);
          
          }
        }
      };
      
      event_sub_ = this->create_subscription<dvs_msgs::msg::EventArray>("/dvs/events", 10, event_callback);
      img_pub_ = it_.advertise("/convolved_img", 10);
      time_map_pub_ = it_.advertise("/time_map", 10);
    }
    
    private:
      rclcpp::Node::SharedPtr node_handle_;
      image_transport::ImageTransport it_;
      
      int event_accum_ = 1;
      cv::Mat cv_img_;
      cv::Mat cv_time_img_;
      double cutoff_freq_;
      int kernel_int_;
      cv::Mat conv_kernel_;

      const cv::Mat identity_k = (cv::Mat_<float>(3,3) << 0, 0, 0, 0, 1, 0, 0, 0, 0);
      const cv::Mat sobel_x = (cv::Mat_<float>(3,3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
      const cv::Mat sobel_y = (cv::Mat_<float>(3,3) << -1, -2, -1, 0, 0, 0, 1, 2, 1);
      const float g = 0.111111f; // = 1/9
      const cv::Mat gaussian = (cv::Mat_<float>(3,3) << g, g, g, g, g, g, g, g, g);
      const cv::Mat laplacian = (cv::Mat_<float>(3,3) << 1, 2, 1, 2, -12, 2, 1, 2, 1);

      void publishState(double t) {
        // decay whole image
        for (int i = 0; i < cv_img_.rows; i++) {
          for (int j = 0; j < cv_img_.cols; j++) {
            double beta = std::exp(-cutoff_freq_ * (float) (t - cv_time_img_.at<double>(i,j)));
            cv_img_.at<float>(i,j) *= beta;
            cv_time_img_.at<double>(i,j) = t;
          }
        }

        cv_bridge::CvImage img;
        double rmin, rmax;
        minMaxLocRobust(cv_img_, rmin, rmax, 5);
        cv_img_ = (cv_img_ - rmin) / (rmax - rmin) * 255.;
        
        cv_img_.convertTo(img.image, CV_8U);
        img.encoding = "mono8";
        img_pub_.publish(img.toImageMsg());
      };

      void switchKernel() {
        kernel_int_ = this->get_parameter("kernel").as_int();
        switch (kernel_int_) {
          case 0:
            conv_kernel_ = identity_k;
            break;
          case 1:
            conv_kernel_ = sobel_x;
            break;
          case 2:
            conv_kernel_ = sobel_y;
            break;
          case 3:
            conv_kernel_ = gaussian;
            break;
          case 4:
            conv_kernel_ = laplacian;
            break;
        }
      }

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
  rclcpp::spin(std::make_shared<IntegratorConv>());
  rclcpp::shutdown();
  return 0;
}
