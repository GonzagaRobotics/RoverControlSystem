#include "Core.h"

using namespace rclcpp;

Core::Core() : Node("rcs_core")
{
    using namespace std::placeholders;

    heartbeatCheckTimer =
        this->create_wall_timer(HEARTBEAT_CHECK_INTERVAL, std::bind(&Core::heartbeatCheckTimerCallback, this));

    killswitchPublisher = this->create_publisher<Killswitch>("/killswitch", 10);

    heartbeatSubscription = this->create_subscription<Heartbeat>(
        "/heartbeat/rover", 10, std::bind(&Core::heartbeatSubscriberCallback, this, _1));

    heartbeatPublisher = this->create_publisher<Heartbeat>("/heartbeat/client", 10);

    sharedSettingsSubscription = this->create_subscription<SharedSettings>(
        "/shared_settings", 10, std::bind(&Core::sharedSettingsSubscriberCallback, this, _1));

    confirmStartSubscription = this->create_subscription<ConfirmStart>(
        "/confirm_start", 10, std::bind(&Core::confirmStartSubscriberCallback, this, _1));

    confirmStopSubscription = this->create_subscription<ConfirmStop>(
        "/confirm_stop", 10, std::bind(&Core::confirmStopSubscriberCallback, this, _1));
}

void Core::heartbeatCheckTimerCallback()
{
    // If the killswitch has been sent, we don't need to check for heartbeat anymore.
    // If the core is not active, we don't need to check for heartbeat either.
    if (!active)
    {
        return;
    }

    Time currentTime = this->get_clock()->now();
    Time lastHeartbeatTimeout = lastHeartbeatTime + std::chrono::duration<float>(heartbeatTimeout);
    auto heartbeatTimeoutOffset = std::chrono::duration<float>(heartbeatTimeoutCount * heartbeatTimeout);

    // A timeout occurs when the current time is greater than the last heartbeat time plus the heartbeat timeout.
    if (currentTime > lastHeartbeatTimeout)
    {
        // Because we check for the heartbeat much more frequently than the heartbeat interval, we might
        // think we have multiple timeouts before we should. So we need to offset the time by the number of
        // timeouts we have had.
        if (currentTime < lastHeartbeatTimeout + heartbeatTimeoutOffset)
        {
            return;
        }

        // We can confirm a timeout has occurred.
        heartbeatTimeoutCount++;

        RCLCPP_WARN(this->get_logger(), "Heartbeat timeout occurred. Count: %d", heartbeatTimeoutCount);

        // If the number of timeouts exceeds the limit, we should send the killswitch.
        if (heartbeatTimeoutCount >= heartbeatTimeoutLimit)
        {
            RCLCPP_WARN(this->get_logger(), "Heartbeat timeout limit hit. Sending killswitch...");

            killswitchPublisher->publish(Killswitch{true});
            active = false;
        }
    }
}

void Core::heartbeatSubscriberCallback(const Heartbeat msg)
{
    // We should only receive heartbeat messages when the core is active.
    if (!active)
    {
        RCLCPP_WARN(this->get_logger(), "Received heartbeat message when core is not active.");
        return;
    }

    // Update the last heartbeat time, the last heartbeat ID, and the heartbeat fail count.
    lastHeartbeatTime = this->get_clock()->now();
    heartbeatTimeoutCount = 0;

    // Send a heartbeat message to the heartbeat topic.
    auto publishedMsg = rcs_interfaces::msg::Heartbeat();
    publishedMsg.source = "rover";
    publishedMsg.sent_time = this->get_clock()->now();
    publishedMsg.id = msg.id;

    heartbeatPublisher->publish(publishedMsg);
}

void Core::sharedSettingsSubscriberCallback(const SharedSettings msg)
{
    // If the core is active, we don't need to do anything.
    if (active)
    {
        RCLCPP_WARN(this->get_logger(), "Received shared settings message when core is already active.");
        return;
    }

    // Set our parameters based on the received message.
    heartbeatInterval = msg.heartbeatInterval;
    heartbeatTimeout = msg.heartbeatTimeout;
    heartbeatTimeoutLimit = msg.heartbeatTimeoutLimit;

    RCLCPP_INFO(this->get_logger(), "Received shared settings.");
}

void Core::confirmStartSubscriberCallback(const ConfirmStart)
{
    // If the core is already active, we don't need to do anything.
    if (active)
    {
        RCLCPP_WARN(this->get_logger(), "Received confirm start message when core is already active.");
        return;
    }

    // Update the last heartbeat time to the current time to avoid a potential timeout.
    lastHeartbeatTime = this->get_clock()->now();

    // The rover is now active.
    active = true;

    RCLCPP_INFO(this->get_logger(), "Core is now active.");
}

void Core::confirmStopSubscriberCallback(const ConfirmStop)
{
    // If the core is not active, we don't need to do anything.
    if (!active)
    {
        RCLCPP_WARN(this->get_logger(), "Received confirm end message when core is not active.");
        return;
    }

    // The rover is no longer active.
    active = false;

    RCLCPP_INFO(this->get_logger(), "Core is no longer active.");
}