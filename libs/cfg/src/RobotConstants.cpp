#include "RobotConstants.h"

#ifdef BUILD_ON_PI
std::string cfg::RobotConstants::shared_serial_port_name = "/dev/ttyUSB0";
#else
std::string cfg::RobotConstants::shared_serial_port_name = "null";
#endif