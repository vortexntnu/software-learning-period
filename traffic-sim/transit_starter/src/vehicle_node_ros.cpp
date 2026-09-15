#include "transit_starter/vehicle_node_ros.hpp"

#include <chrono>
#include <stdexcept>

VehicleNode::VehicleNode() : Node("vehicle_node") {
    set_parameters();

    set_subscribers_and_publisher();
}

// ---------------------------------------------------------------------------
// Written for you. Read it, but you do not have to change it.
//
// Every value the node needs is a ROS parameter with a default. The real
// values are in config/transit_params.yaml, so you can change your lane or
// your speed there and just relaunch, without rebuilding. declare_parameter
// registers the name and hands back either the value from that file or the
// default written here.
// ---------------------------------------------------------------------------
void VehicleNode::set_parameters() {
    vehicle_topic_ = this->declare_parameter<std::string>(
        "topics.vehicle_state", "/vehicle_state");
    signal_topic_ = this->declare_parameter<std::string>(
        "topics.signal_state", "/signal_state");

    vehicle_id_ = this->declare_parameter<std::string>("vehicle.vehicle_id",
                                                       "car_yourname");
    lane_id_ =
        static_cast<uint16_t>(this->declare_parameter<int>("vehicle.lane_id", 1));
    color_ = this->declare_parameter<std::string>("vehicle.color", "red");

    speed_ = this->declare_parameter<double>("vehicle.speed", 8.0);
    lane_length_ = this->declare_parameter<double>("vehicle.lane_length", 80.0);
    stop_progress_ =
        this->declare_parameter<double>("vehicle.stop_progress", 0.5);

    tick_hz_ = this->declare_parameter<double>("vehicle.tick_hz", 20.0);
}

// ---------------------------------------------------------------------------
// Written for you. This runs tick_hz_ times a second, once you have created
// the timer in set_subscribers_and_publisher() below.
//
// It calls three functions, one per task. They already exist further down
// this file, and they are all empty. Fill them in one at a time, in the
// order the curriculum gives, and the car does a little more each time.
// ---------------------------------------------------------------------------
void VehicleNode::tick() {
    if (must_stop_for_light()) {
        // Task 5 makes this happen. Until then it never runs.
        moving_ = false;
        velocity_ = 0.0;
    } else {
        advance_progress();
    }

    publish_state();
}

void VehicleNode::set_subscribers_and_publisher() {
    // TODO (Task 1): create the publisher and the timer, then delete the
    //     throw at the bottom of this function.
    //
    //     The publisher sends messages of type
    //     transit_msgs::msg::VehicleState on the topic named by
    //     vehicle_topic_, and has to be stored in vehicle_pub_.
    //
    //     The timer has to call this node's tick() every
    //     1.0 / tick_hz_ seconds, and has to be stored in timer_.
    //
    //     The shape of both calls is in transit_starter/README.md, under
    //     "The ROS 2 calls you will need". Write the timer period as
    //     std::chrono::duration<double>(1.0 / tick_hz_).

    vehicle_pub_ = this->create_publisher<transit_msgs::msg::VehicleState>(
        vehicle_topic_, 10);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / tick_hz_),
        std::bind(&VehicleNode::tick, this));

    // TODO (Task 5): create the subscription. Leave this until Tasks 1 and 2
    //     work and you are ready to obey the light.
    //
    //     It listens for messages of type transit_msgs::msg::SignalState on
    //     the topic named by signal_topic_, calls this node's on_signal for
    //     every message, and has to be stored in signal_sub_.
    //
    //     The shape of the call is in transit_starter/README.md, under
    //     "The ROS 2 calls you will need".
    //
    //     Note you will receive every traffic light in the city on this
    //     topic, not only yours. Sorting that out is on_signal's job.
}

void VehicleNode::publish_state() {
    // TODO (Task 1): build one VehicleState message, fill in its fields,
    //     publish it with vehicle_pub_, then delete the throw below.
    //
    //     Make an empty message like this:
    //         transit_msgs::msg::VehicleState message;
    //
    //     Then set its six fields. They are described in
    //     transit_msgs/msg/VehicleState.msg:
    //         message.vehicle_id = vehicle_id_;
    //         message.lane_id    = lane_id_;
    //         message.progress   = progress_;
    //         message.color      = color_;
    //         message.velocity   = velocity_;
    //         message.moving     = moving_;
    //
    //     Then hand it to the publisher: vehicle_pub_->publish(message).
    //
    //     You do not need to touch this function again after Task 1. The
    //     later tasks change progress_, moving_ and velocity_, and this
    //     function just sends whatever they are at the time.

    transit_msgs::msg::VehicleState message;

    message.vehicle_id = vehicle_id_;
    message.lane_id = lane_id_;
    message.progress = progress_;
    message.color = color_;
    message.velocity = velocity_;
    message.moving = moving_;

    vehicle_pub_->publish(message);

}

void VehicleNode::advance_progress() {
    // TODO (Task 2): move the car a little further along its lane.
    //
    //     progress_ is not a distance. It is a fraction of the lane: 0.0 at
    //     the start, 1.0 at the end. So work out how much of the lane the
    //     car covers between one call and the next, and add that.
    //
    //     What you have to work with: speed_ is in metres per second,
    //     lane_length_ is in metres, and tick_hz_ is how many times a
    //     second this function is called. Getting from those three to "a
    //     fraction of the lane" is the task. Check the units as you go.
    //
    //     When progress_ runs past 1.0 the car is at the end of the lane
    //     and should carry on from the start.
    //
    //     Set moving_ and velocity_ too, or the map keeps drawing the car
    //     as parked.
    //
    //     This function being empty is what makes the car stand still in
    //     Task 1, so there is no throw to delete here.

    progress_ += (speed_ / lane_length_) / tick_hz_;
    if (progress_ > 1.0) {
        progress_ -= 1.0;
    }

    moving_ = true;
    velocity_ = speed_;
}

bool VehicleNode::must_stop_for_light() {
    // TODO (Task 5): return true when the car has to wait, false when it
    //     is free to drive. Replace the `return false;` below.
    //
    //     The obvious version is wrong. Stopping whenever the light is red
    //     freezes the car wherever it happens to be standing, which is
    //     usually in the middle of the junction. A car stops AT THE STOP
    //     LINE, so where the car is matters as much as what colour the
    //     light is. stop_progress_ is where that line sits, in the same
    //     0.0-to-1.0 units as progress_.
    //
    //     on_signal() below puts the latest colour in light_state_. The
    //     values to compare it against are
    //     transit_msgs::msg::SignalState::RED, ::YELLOW and ::GREEN.
    //
    //     Two things to decide for yourself. Before any light message has
    //     arrived light_state_ is -1, which is not a colour. And you are
    //     really deciding "may I enter the junction", which is not quite
    //     the same question as "is the light red".
    //
    //     Returning false always, as it does now, means the car ignores
    //     lights completely. That is correct for Tasks 1 to 4.

    return false;
}

void VehicleNode::on_signal(const transit_msgs::msg::SignalState::SharedPtr msg) {
    // TODO (Task 5): remember the state of the light on our lane.
    //
    //     This runs for every traffic light in the city, not only ours, so
    //     the first thing to do is check whether this message is even about
    //     us: msg->lane_id is the lane the light governs, and lane_id_ is
    //     the lane we drive on. If they are different, return and do
    //     nothing.
    //
    //     If they match, store msg->state in light_state_, which
    //     must_stop_for_light() above reads.
    //
    //     You read fields off a message with -> here, not with a dot,
    //     because msg arrives as a pointer.

    (void)msg;  // delete this line once you use msg
}
