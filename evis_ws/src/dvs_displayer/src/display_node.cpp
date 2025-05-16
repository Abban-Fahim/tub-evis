#include <dvs_displayer/display.h>

class Displayer : public rclcpp::Node {
  public:
    Displayer() : rclcpp::Node("displayer"), node_handle_(std::shared_ptr<Displayer>(this, [](auto *) {})), it_(node_handle_) {

      this->declare_parameter("mode", "gray");
      this->declare_parameter("cmap", 0);

      auto event_callback = [this](dvs_msgs::msg::EventArray::SharedPtr msg) -> void {
        std::string mode_value = this->get_parameter("mode").as_string();        
        RCLCPP_INFO(this->get_logger(), "meow %s", mode_value.c_str());
        
        cv::Mat cv_img;

        cv::Mat pos_values = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(0));
        cv::Mat neg_values = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(0));
        cv::Mat gray_img = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(128));
        
        for (dvs_msgs::msg::Event e: msg->events)
        {
          if (e.polarity) {
            pos_values.at<uint8_t>(e.y, e.x)++;
          } else {
            neg_values.at<uint8_t>(e.y, e.x)++;
          }
        }
        
        double max_pos, max_neg, dummy;
        cv::minMaxLoc(pos_values, &dummy, &max_pos);
        cv::minMaxLoc(neg_values, &dummy, &max_neg);
        const double scale_val = 127 / std::max(max_pos, max_neg);
          
        gray_img += scale_val * pos_values;
        gray_img -= scale_val * neg_values;

        if (mode_value == "gray") {
          cv_img = gray_img;
        } else {
          int cmap_val = this->get_parameter("cmap").as_int();
          if (img_recieved_) {
            last_img_.copyTo(cv_img);
          } else {
            cv_img = cv::Mat(msg->height, msg->width, CV_8UC3, cv::Scalar(255, 255, 255));
          }

          cv::Mat cmap_img;
          cv::applyColorMap(gray_img, cmap_img, cmap_val);

          // Replace image pixel
          for (int i = 0; i < cv_img.rows; i++) {
            for (int j = 0; j < cv_img.cols; j++) {
              if (gray_img.at<uint8_t>(i, j) != 128) {
                cv_img.at<cv::Vec3b>(i, j) = cmap_img.at<cv::Vec3b>(i, j);
              }
            }
          }
        }
          
        cv_bridge::CvImage img;
        cv_img.copyTo(img.image);
        img.encoding = mode_value == "gray" ? "mono8" : "bgr8";
        img_pub_.publish(img.toImageMsg());
      };

      auto img_callback = [this](sensor_msgs::msg::Image::SharedPtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "meowww %d", msg->width);
        try
        {
          cv::cvtColor(cv_bridge::toCvCopy(msg)->image, last_img_, cv::COLOR_GRAY2BGR);
          img_recieved_ = true;
        }
        catch (cv_bridge::Exception& e)
        {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
        }
      };


      event_sub_ = this->create_subscription<dvs_msgs::msg::EventArray>("/dvs/events", 10, event_callback);
      img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/reconstructed_img", 10, img_callback);
      img_pub_ = it_.advertise("/combined_img", 10);
    }
  
  private:
    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport it_;

    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    image_transport::Publisher img_pub_;

    cv::Mat last_img_;
    bool img_recieved_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Displayer>());
  rclcpp::shutdown();
  return 0;
}
