#ifndef TB6612_DRIVE_HH
#define TB6612_DRIVE_HH

#include <cstring>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

/*

    A hardware interface communicate with two joints.
    It takes arguments:
        GPIO numbers:
        - MOTORA1
        - MOTORA2
        - MOTORB1
        - MOTORB2

        Joints names
        
    

*/

namespace tb612_drive
{
    class TB612Drive : public hardware_interface::SysemInterface
    {
        public:

        TB612Drive();

        CallbackReturn on_configure(const State &previous_state);

        CallbackReturn on_cleanup(const State &previous_state);

        CallbackReturn on_shutdown(const State &previous_state);

        CallbackReturn on_activate(const State &previous_state);

        CallbackReturn on_error(const State &previous_state);

        CallbackReturn on_init(const HardwareInfo &hardware_info);

        std::vector<hardware_interface::StateInterface> export_state_interfaces();

        std::vector<hardware_interface::CommandInterface> export_command_interfaces();

        return_type read(const rclcpp::Time &time, const rclcpp::Duration &period);

        return_type write(const rclcpp::Time &time, const rclcpp::Duration &period);

        private:

        rclcpp::Logger logger_;
    };
}

#endif