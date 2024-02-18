#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rcs_interfaces/srv/shared_config.hpp"

using namespace rclcpp;
using namespace std::chrono_literals;
using namespace std::placeholders;

const auto heartbeatCheckInterval = 500ms;

class RCSCore : public Node
{
private:
    float heartbeatInterval = 0;
    float heartbeatTimeout = 0;
    int heartbeatFailLimit = 0;

    Time lastHeartbeatTime;
    int heartbeatFailCount = 0;

    bool sentKillswitch = false;

    Service<rcs_interfaces::srv::SharedConfig>::SharedPtr sharedConfigService;

    Subscription<std_msgs::msg::Empty>::SharedPtr heartbeatSubscription;
    Publisher<std_msgs::msg::Empty>::SharedPtr heartbeatPublisher;
    TimerBase::SharedPtr heartbeatCheckTimer;

    Publisher<std_msgs::msg::Empty>::SharedPtr killswitchPublisher;

    void sharedConfigCallback(
        const std::shared_ptr<rcs_interfaces::srv::SharedConfig::Request> request,
        std::shared_ptr<rcs_interfaces::srv::SharedConfig::Response> response)
    {
        heartbeatInterval = request->heartbeat_interval;
        heartbeatTimeout = request->heartbeat_timeout;
        heartbeatFailLimit = request->heartbeat_fail_limit;

        response->success = true;

        RCLCPP_INFO(this->get_logger(), "Shared config set.");
    }

    void heartbeatSubscriberCallback(const std_msgs::msg::Empty::SharedPtr)
    {
        RCLCPP_INFO(this->get_logger(), "Heartbeat received.");

        lastHeartbeatTime = this->get_clock().get()->now();
        heartbeatFailCount = 0;
        sentKillswitch = false;

        heartbeatPublisher->publish(std_msgs::msg::Empty());
    }

    void heartbeatCheckTimerCallback()
    {
        // This timer might be called before we get the shared config, so we need to check if it's set.
        // For now, we will check if the heartbeat interval is set to 0, which is the default value.
        if (heartbeatInterval == 0)
        {
            return;
        }

        // If we already sent the killswitch, we don't need to check for heartbeat anymore.
        if (sentKillswitch)
        {
            return;
        }

        Time currentTime = this->get_clock().get()->now();

        // The interval we're checking is less than the interval that's being sent, meaning that we will fail and
        // send the killswitch faster than we should. To mitigate this, we will add time to the last heartbeat time
        // based on the interval and number of fails.

        // Trying to find a weird bug in here
        try
        {
            auto adjustedLastTime = lastHeartbeatTime + Duration::from_seconds(heartbeatInterval * heartbeatFailCount);

            if (currentTime - adjustedLastTime > Duration::from_seconds(heartbeatTimeout))
            {
                RCLCPP_WARN(this->get_logger(), "Heartbeat timeout %d.", ++heartbeatFailCount);

                if (heartbeatFailCount > heartbeatFailLimit)
                {
                    RCLCPP_ERROR(this->get_logger(), "Heartbeat fail limit exceeded. Activating killswitch....");

                    sentKillswitch = true;
                    killswitchPublisher->publish(std_msgs::msg::Empty());
                }
            }
        }
        catch (std::runtime_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error checking heartbeat: %s", e.what());
        }
    }

public:
    RCSCore() : Node("rcs_core")
    {
        sharedConfigService = this->create_service<rcs_interfaces::srv::SharedConfig>(
            "shared_config",
            std::bind(&RCSCore::sharedConfigCallback, this, _1, _2));

        killswitchPublisher = this->create_publisher<std_msgs::msg::Empty>("killswitch", 10);

        heartbeatPublisher = this->create_publisher<std_msgs::msg::Empty>("heartbeat/client", QoS(10));

        heartbeatSubscription = this->create_subscription<std_msgs::msg::Empty>(
            "heartbeat/rover",
            QoS(10),
            std::bind(&RCSCore::heartbeatSubscriberCallback, this, _1));

        heartbeatCheckTimer = this->create_wall_timer(
            heartbeatCheckInterval,
            std::bind(&RCSCore::heartbeatCheckTimerCallback, this));
    }
};

int main(int argc, char *argv[])
{
    init(argc, argv);

    spin(std::make_shared<RCSCore>());

    shutdown();

    return 0;
}