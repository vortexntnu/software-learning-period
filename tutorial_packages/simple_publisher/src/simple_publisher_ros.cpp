#include "simple_publisher/simple_publisher_ros.hpp"

using namespace std::chrono_literals;

SimplePublisher::SimplePublisher() : Node("simple_publisher")
{
    // Ros2 parameters can be declared like this. 
    // The parameters intelligently are set to the values specified in the param file.
    // We are able to declare parameters with default values here,
    // but we prefer not to do that, to ensure the values from the parameter file are used
    // in the case we have misspelled a parameter name or something like that.
    this->declare_parameter<std::string>("publish_topic");
    this->declare_parameter<std::string>("publish_value");

    // Get the values of the parameters. the values are set in the parameter file.
    std::string topic = this->get_parameter("publish_topic").as_string();
    publish_value_ = this->get_parameter("publish_value").as_string();

    // In this tutorial we will introduce the concept of Quality of Service (QoS) profiles.
    // QoS profiles allow you to specify how messages are sent and received over the network with ros2.

    // publisher_ = this->create_publisher<std_msgs::msg::String>(topic, 10);

    // When you pass the number 10 to the create_publisher method, what happens is that a qos profile
    // with a history depth of 10 is created with a reliability set to reliable.
    // A history depth of 10 means that the publisher will keep the last 10 messages in memory,
    // and if a subscriber connects, it will receive the last 10 messages.
    // Reliable means that the publisher will ensure that messages are delivered to subscribers.
    // This requires a two-way communication between the publisher and subscriber,
    // which can introduce some latency, but it ensures that messages are not lost.

    // Sometimes latency is more important than ensuring all messages are delivered.
    // A common use case is when you are publishing sensor data, like camera images.

    // In this case, you can use a QoS profile with a history depth of 1 and best-effort reliability.
    // This means that the publisher will only keep the last message in memory,
    // and messages might be delivered faster, at the cost of potentially losing some messages.
    // This is useful for real-time applications where you want to receive
    // the most recent data as quickly as possible, even if it means that some messages might be lost.



    // Create a QoS profile with history depth of 1 and best-effort reliability.
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // Create the publisher with the QoS profile.
    // You can verify that the QoS profile is set correctly by checking the QoS settings of the publisher.
    // ros2 topic info --verbose /topic_name 
    // This will show you the QoS settings of the publisher.
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic, qos_profile);

    timer_ = this->create_wall_timer(
        1s, std::bind(&SimplePublisher::timer_callback, this));
}

void SimplePublisher::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = publish_value_;
    publisher_->publish(message);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}
