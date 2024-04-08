#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rcs_interfaces/msg/shared_settings.hpp"
#include "rcs_interfaces/msg/confirm_start.hpp"
#include "rcs_interfaces/msg/confirm_stop.hpp"
#include "rcs_interfaces/msg/heartbeat.hpp"
#include "rcs_interfaces/msg/killswitch.hpp"

struct SharedSettings
{
    float heartbeatInterval;
    float heartbeatTimeout;
    uint32_t heartbeatTimeoutLimit;
};

struct ConfirmStart
{
};

struct ConfirmStop
{
};

struct Heartbeat
{
    rclcpp::Time sentTime;
    std::string source;
    uint32_t id;
};

struct Killswitch
{
    bool enable;
};

template <>
struct rclcpp::TypeAdapter<SharedSettings, rcs_interfaces::msg::SharedSettings>
{
    using is_specialized = std::true_type;
    using custom_type = SharedSettings;
    using ros_message_type = rcs_interfaces::msg::SharedSettings;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.heartbeat_interval = source.heartbeatInterval;
        destination.heartbeat_timeout = source.heartbeatTimeout;
        destination.heartbeat_timeout_limit = source.heartbeatTimeoutLimit;
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination.heartbeatInterval = source.heartbeat_interval;
        destination.heartbeatTimeout = source.heartbeat_timeout;
        destination.heartbeatTimeoutLimit = source.heartbeat_timeout_limit;
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(SharedSettings, rcs_interfaces::msg::SharedSettings);

template <>
struct rclcpp::TypeAdapter<ConfirmStart, rcs_interfaces::msg::ConfirmStart>
{
    using is_specialized = std::true_type;
    using custom_type = ConfirmStart;
    using ros_message_type = rcs_interfaces::msg::ConfirmStart;

    static void convert_to_ros_message(const custom_type &, ros_message_type &)
    {
    }

    static void convert_to_custom(const ros_message_type &, custom_type &)
    {
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(ConfirmStart, rcs_interfaces::msg::ConfirmStart);

template <>
struct rclcpp::TypeAdapter<ConfirmStop, rcs_interfaces::msg::ConfirmStop>
{
    using is_specialized = std::true_type;
    using custom_type = ConfirmStop;
    using ros_message_type = rcs_interfaces::msg::ConfirmStop;

    static void convert_to_ros_message(const custom_type &, ros_message_type &)
    {
    }

    static void convert_to_custom(const ros_message_type &, custom_type &)
    {
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(ConfirmStop, rcs_interfaces::msg::ConfirmStop);

template <>
struct rclcpp::TypeAdapter<Heartbeat, rcs_interfaces::msg::Heartbeat>
{
    using is_specialized = std::true_type;
    using custom_type = Heartbeat;
    using ros_message_type = rcs_interfaces::msg::Heartbeat;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.sent_time = source.sentTime;
        destination.source = source.source;
        destination.id = source.id;
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination.sentTime = source.sent_time;
        destination.source = source.source;
        destination.id = source.id;
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Heartbeat, rcs_interfaces::msg::Heartbeat);

template <>
struct rclcpp::TypeAdapter<Killswitch, rcs_interfaces::msg::Killswitch>
{
    using is_specialized = std::true_type;
    using custom_type = Killswitch;
    using ros_message_type = rcs_interfaces::msg::Killswitch;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.enable = source.enable;
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination.enable = source.enable;
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Killswitch, rcs_interfaces::msg::Killswitch);