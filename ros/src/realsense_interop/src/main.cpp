#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/image_transport.hpp"

using namespace rclcpp;

class RealsenseInterop : public Node
{
private:
	image_transport::CameraSubscriber cameraSubscriber;
	image_transport::Publisher cameraPublisher;

	TimerBase::SharedPtr timer;

	sensor_msgs::msg::Image::ConstSharedPtr lastImage;

	void cameraSubscriberCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr image,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr)
	{
		lastImage = image;
	}

	void timerCallback()
	{
		if (lastImage)
		{
			cameraPublisher.publish(lastImage);
			lastImage.reset();

			if (lastImage.use_count() > 0)
			{
				RCLCPP_WARN(get_logger(), "lastImage still has %ld references", lastImage.use_count());
			}
		}

		// auto image = std::make_shared<sensor_msgs::msg::Image>();
		// image->header.stamp = now();
		// image->header.frame_id = "testing";
		// image->height = 480;
		// image->width = 640;
		// image->encoding = "rgb8";
		// image->is_bigendian = false;
		// image->step = 640 * 3;

		// // Generate random image data
		// image->data.resize(image->step * image->height);
		// std::random_device rd;

		// for (size_t i = 0; i < image->data.size(); i += 3)
		// {
		//     image->data[i] =
		//         image->data[i + 1] =
		//             image->data[i + 2] = rd() % 256;
		// }

		// cameraPublisher.publish(image);
	}

public:
	RealsenseInterop() : Node("realsense_interop")
	{
		using namespace std::placeholders;

		cameraSubscriber = image_transport::create_camera_subscription(
			this,
			"camera/color/image_raw",
			std::bind(&RealsenseInterop::cameraSubscriberCallback, this, _1, _2),
			"raw",
			QoS(2).get_rmw_qos_profile());

		cameraPublisher = image_transport::create_publisher(
			this,
			"realsense_interop/image",
			QoS(1).get_rmw_qos_profile());

		timer = this->create_wall_timer(
			std::chrono::milliseconds(250),
			std::bind(&RealsenseInterop::timerCallback, this));
	}
};

int main(int argc, char *argv[])
{
	init(argc, argv);

	spin(std::make_shared<RealsenseInterop>());

	shutdown();

	return 0;
}