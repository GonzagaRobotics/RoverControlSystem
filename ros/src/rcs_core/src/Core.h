#pragma once

#include <functional>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rcs_interfaces/Types.h"

const auto HEARTBEAT_CHECK_INTERVAL = std::chrono::milliseconds(500);

class Core : public rclcpp::Node
{
private:
    float heartbeatInterval = 0;
    float heartbeatTimeout = 0;
    unsigned int heartbeatTimeoutLimit = 0;

    rclcpp::Time lastHeartbeatTime;
    unsigned int heartbeatTimeoutCount = 0;

    bool active = false;

    rclcpp::TimerBase::SharedPtr heartbeatCheckTimer;
    rclcpp::Publisher<Killswitch>::SharedPtr killswitchPublisher;

    void heartbeatCheckTimerCallback();

    rclcpp::Subscription<Heartbeat>::SharedPtr heartbeatSubscription;
    rclcpp::Publisher<Heartbeat>::SharedPtr heartbeatPublisher;

    void heartbeatSubscriberCallback(const Heartbeat);

    rclcpp::Subscription<SharedSettings>::SharedPtr sharedSettingsSubscription;
    rclcpp::Subscription<ConfirmStart>::SharedPtr confirmStartSubscription;
    rclcpp::Subscription<ConfirmStop>::SharedPtr confirmStopSubscription;

    void sharedSettingsSubscriberCallback(const SharedSettings);
    void confirmStartSubscriberCallback(const ConfirmStart);
    void confirmStopSubscriberCallback(const ConfirmStop);

public:
    Core();
};