#include <dvs_displayer/display.h>

class Displayer : public rclcpp::Node {
  public:
    Displayer() : rclcpp::Node("displayer"), node_handle_(std::shared_ptr<Displayer>(this, [](auto *) {})), it_(node_handle_) {

      // cv::Mat cv_img;

      auto event_callback = [this](dvs_msgs::msg::EventArray::SharedPtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "meow %ld", msg->events.size());
        
        cv::Mat cv_img = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(128));
        cv::Mat pos_values = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(0));
        cv::Mat neg_values = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(0));
        
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

        cv_img += scale_val * pos_values;
        cv_img -= scale_val * neg_values;

        cv_bridge::CvImage img;
        cv_img.copyTo(img.image);
        img.encoding = "mono8";
        img_pub_.publish(img.toImageMsg());
      };

      auto img_callback = [this](sensor_msgs::msg::Image::SharedPtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "meowww %d", msg->width);

        cv::Mat cv_img;
        try
        {
          cv::cvtColor(cv_bridge::toCvCopy(msg)->image, cv_img, cv::COLOR_GRAY2BGR);
        }
        catch (cv_bridge::Exception& e)
        {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
        }
      };


      event_sub_ = this->create_subscription<dvs_msgs::msg::EventArray>("/dvs/events", 10, event_callback);
      img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/dvs/image_raw", 10, img_callback);
      img_pub_ = it_.advertise("/combined_img", 10);

    }
  
  private:
    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport it_;

    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    image_transport::Publisher img_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Displayer>());
  rclcpp::shutdown();
  return 0;
}
