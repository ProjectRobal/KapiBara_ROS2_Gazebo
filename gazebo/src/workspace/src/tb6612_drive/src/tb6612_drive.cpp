#include "pluginlib/class_list_macros.hpp"
#include "tb6612_drive.hpp"

namespace tb612_drive
{
    TB612Drive::TB612Drive()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
    {}

    CallbackReturn TB612Drive::on_configure(const State &previous_state)
    {

    }

    CallbackReturn TB612Drive::on_cleanup(const State &previous_state)
    {

    }

    CallbackReturn TB612Drive::on_shutdown(const State &previous_state)
    {

    }

    CallbackReturn TB612Drive::on_activate(const State &previous_state)
    {

    }

    CallbackReturn TB612Drive::on_error(const State &previous_state)
    {

    }

    CallbackReturn TB612Drive::on_init(const HardwareInfo &hardware_info)
    {

    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces()
    {

    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces()
    {
        
    }

    return_type TB612Drive::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

    }

    return_type TB612Drive::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

    }

    PLUGINLIB_EXPORT_CLASS
}