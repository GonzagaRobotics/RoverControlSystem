#pragma once

#include <functional>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcs_interfaces/msg/confirm_start.hpp"
#include "rcs_interfaces/msg/confirm_stop.hpp"
#include "rcs_interfaces/msg/heartbeat.hpp"
#include "rcs_interfaces/msg/killswitch.hpp"

const auto HEARTBEAT_CHECK_INTERVAL = std::chrono::milliseconds(500);

class Core : public rclcpp::Node
{
private:
    float heartbeatInterval = 0;
    float heartbeatTimeout = 0;
    unsigned int heartbeatTimeoutLimit = 0;

    rclcpp::Time lastHeartbeatTime;
    unsigned int lastHeartbeatId = 0;
    unsigned int heartbeatTimeoutCount = 0;

    bool active = false;

    rclcpp::TimerBase::SharedPtr heartbeatCheckTimer;
    rclcpp::Publisher<rcs_interfaces::msg::Killswitch>::SharedPtr killswitchPublisher;

    void heartbeatCheckTimerCallback();

    rclcpp::Subscription<rcs_interfaces::msg::Heartbeat>::SharedPtr heartbeatSubscription;
    rclcpp::Publisher<rcs_interfaces::msg::Heartbeat>::SharedPtr heartbeatPublisher;

    void heartbeatSubscriberCallback(const rcs_interfaces::msg::Heartbeat::SharedPtr);

    rclcpp::Subscription<rcs_interfaces::msg::ConfirmStart>::SharedPtr confirmStartSubscription;
    rclcpp::Subscription<rcs_interfaces::msg::ConfirmStop>::SharedPtr confirmStopSubscription;

    void confirmStartSubscriberCallback(const rcs_interfaces::msg::ConfirmStart::SharedPtr);
    void confirmStopSubscriberCallback(const rcs_interfaces::msg::ConfirmStop::SharedPtr);

public:
    Core();
};