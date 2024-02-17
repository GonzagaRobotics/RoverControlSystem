#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace rclcpp;

class RealsenseInterop : public Node
{
private:
    Subscription<sensor_msgs::msg::Image>::SharedPtr cameraSubscriber;

    void cameraSubscriberCallback(const sensor_msgs::msg::Image::SharedPtr)
    {
        RCLCPP_INFO(this->get_logger(), "Camera image received.");
    }

public:
    RealsenseInterop() : Node("realsense_interop")
    {
        using namespace std::placeholders;

        cameraSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/color/image_raw",
            QoS(10),
            std::bind(&RealsenseInterop::cameraSubscriberCallback, this, _1));
    }
};

int main(int argc, char *argv[])
{
    init(argc, argv);

    spin(std::make_shared<RealsenseInterop>());

    shutdown();

    return 0;
}