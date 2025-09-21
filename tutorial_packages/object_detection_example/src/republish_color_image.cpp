#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class RepublishColorImage : public rclcpp::Node
{
public:
  RepublishColorImage()
  : Node("republish_color_image_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("republishing_image_topic", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/zed_node/left/image_rect_color", 10,
      std::bind(&RepublishColorImage::topic_callback, this, _1));

    // Keep your timer_ variable; make it a no-op to avoid referencing a missing MinimalPublisher
    timer_ = this->create_wall_timer(500ms, [](){}); 
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)  // removed const so we can publish
  {
    try {
      // Convert (or ensure) BGR8 and republish
      cv_bridge::CvImageConstPtr cv_in = cv_bridge::toCvShare(msg);
      cv_bridge::CvImage out(msg->header, sensor_msgs::image_encodings::BGR8, cv_in->image.clone());
      publisher_->publish(*out.toImageMsg());
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  // --- keep your original member names ---
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RepublishColorImage>());
  rclcpp::shutdown();
  return 0;
}
