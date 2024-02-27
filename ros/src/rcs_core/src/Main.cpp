#include "Core.h"

using namespace rclcpp;

int main(int argc, char *argv[])
{
    init(argc, argv);

    spin(std::make_shared<Core>());

    shutdown();

    return 0;
}