#include "ashish_starter/signal_node_ros.hpp"

#include <chrono>
#include <cmath>     // std::fmod, which Task 3 needs
#include <stdexcept>

SignalNode::SignalNode() : Node("signal_node") {
    set_parameters();

    set_publisher();
}

// ---------------------------------------------------------------------------
// Written for you, the same way as in vehicle_node_ros.cpp. Every value is a
// ROS parameter with a default, and the real values live in
// config/transit_params.yaml, so you can retime the lights and relaunch
// without rebuilding.
// ---------------------------------------------------------------------------
void SignalNode::set_parameters() {
    signal_topic_ = this->declare_parameter<std::string>("topics.signal_state",
                                                         "/signal_state");

    lane_id_ =
        static_cast<uint16_t>(this->declare_parameter<int>("signal.lane_id", 1));
    signal_id_ = this->declare_parameter<std::string>("signal.signal_id",
                                                      "junction1_lane1");

    green_seconds_ = this->declare_parameter<double>("signal.green_seconds", 9.0);
    yellow_seconds_ =
        this->declare_parameter<double>("signal.yellow_seconds", 2.0);
    all_red_seconds_ =
        this->declare_parameter<double>("signal.all_red_seconds", 1.0);

    tick_hz_ = this->declare_parameter<double>("signal.tick_hz", 5.0);
}

// ---------------------------------------------------------------------------
// Written for you. This runs tick_hz_ times a second, once you have created
// the timer in set_publisher() below.
//
// It keeps the clock for you. elapsed_ is how many seconds the node has been
// running, and that is the only thing your light logic needs to know: a
// traffic light is just "given the time, which colour am I".
// ---------------------------------------------------------------------------
void SignalNode::tick() {
    elapsed_ += 1.0 / tick_hz_;

    publish_lights();
}

void SignalNode::set_publisher() {
    // TODO (Task 3): create the publisher and the timer, then delete the
    //     throw at the bottom of this function.
    //
    //     The publisher sends messages of type
    //     transit_msgs::msg::SignalState on the topic named by
    //     signal_topic_, and has to be stored in signal_pub_.
    //
    //     The timer has to call this node's tick() every
    //     1.0 / tick_hz_ seconds, and has to be stored in timer_.
    //
    //     This is the same pair you already wrote in Task 1, over in
    //     vehicle_node_ros.cpp. Only the message type and the variable
    //     names change, so read your own code rather than starting again.

    signal_pub_ = this->create_publisher<transit_msgs::msg::SignalState>(signal_topic_, 10);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / tick_hz_),
        std::bind(&SignalNode::tick, this));

    
}

void SignalNode::publish_lights() {
    // TODO (Task 3): publish one SignalState for our own lane, then delete
    //     the throw below.
    //
    //     Fill in a message and publish it with signal_pub_, the same way
    //     you published a VehicleState in Task 1. Its three fields are
    //     listed in transit_msgs/msg/SignalState.msg, and the colour comes
    //     from state_for_lane(lane_id_).
    //
    //     Write state_for_lane() first, just below. This function is
    //     useless without it.

    transit_msgs::msg::SignalState message;
    message.lane_id = lane_id_;
    message.signal_id = signal_id_;
    message.state = state_for_lane(lane_id_);

    signal_pub_->publish(message);


    // TODO (Task 4): publish all four approaches instead of only ours.
    //
    //     The junction has four of them, lanes 1 to 4, and each needs its
    //     own message with its own lane_id, its own signal_id and its own
    //     colour from state_for_lane(). So four messages per tick.
    for (uint16_t lane = 1; lane <= 4; ++lane) {
        transit_msgs::msg::SignalState message;
        message.lane_id = lane;
        message.signal_id = "junction1_lane" + std::to_string(lane);
        message.state = state_for_lane(lane);

        signal_pub_->publish(message);
    }
}

uint8_t SignalNode::state_for_lane(uint16_t lane) {
    // TODO (Task 3): work out which colour a lane shows at this moment, and
    //     return it. Replace the `return transit_msgs::msg::SignalState::RED;`
    //     below.
    //
    //     The three colours you can return are
    //     transit_msgs::msg::SignalState::RED, ::YELLOW and ::GREEN.
    //
    //     A traffic light repeats one round forever: green_seconds_ of
    //     green, then yellow_seconds_ of yellow, then all_red_seconds_ of
    //     red, then round again.
    //
    //     elapsed_ only ever counts up, but a light goes round, so what you
    //     want is where inside the current round elapsed_ falls. The
    //     remainder of a division is what turns one into the other, and
    //     std::fmod (from <cmath>) does that for doubles. From there it is
    //     a matter of comparing against the phase lengths.
    //
    //     For Task 3 you can ignore the `lane` argument and give every lane
    //     the same answer. Only lane_id_ is being published anyway.

    // TODO (Task 4): make the answer depend on `lane`.
    //
    //     Two rules define the junction. Lanes that face each other share a
    //     phase and go green together. Lanes that cross each other must
    //     NEVER be green at the same time — that is a crash, and it is what
    //     this task is really testing. transit_sim/README.md's lane table
    //     says which direction each lane runs, so it tells you the pairs.
    //
    //     The rest is yours to work out: how long a full cycle lasts now
    //     that two pairs take turns, and how you get from a time inside
    //     that cycle to a colour for a given lane. It is the Task 3 round,
    //     one level up.
    //
    //     Compare your cycle against PHASES in scripts/drive_city.py once
    //     it works, not before.


    double cycle = 2 * (green_seconds_ + yellow_seconds_ + all_red_seconds_);
    double half = green_seconds_ + yellow_seconds_ + all_red_seconds_;
    double t = std::fmod(elapsed_, cycle);

    switch(lane) {
        case 1:
        case 2:
            break;

        case 3:
        case 4:
            t = std::fmod(t + half, cycle);
            break;
        default:
            return transit_msgs::msg::SignalState::RED;
    }

    if (t < green_seconds_) {
        return transit_msgs::msg::SignalState::GREEN;
    } else if (t < green_seconds_ + yellow_seconds_) {
        return transit_msgs::msg::SignalState::YELLOW;
    } else {
        return transit_msgs::msg::SignalState::RED;
    }
}
