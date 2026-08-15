// ═══════════════════════════════════════════════════════════════════════════
//  ros2_control SystemInterface plugin — 通用骨架的標頭（ROS 2 Jazzy）
//
//  public 區塊每個機器人都一樣，會變的只有 private。
// ═══════════════════════════════════════════════════════════════════════════

#ifndef MY_HARDWARE__MY_SYSTEM_HPP_
#define MY_HARDWARE__MY_SYSTEM_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_hardware
{

class MySystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MySystem)

  // ── 以下六個是框架唯一認得的入口，每個機器人都一樣 ──────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ═════ 以下全部依機器人修改 ═══════════════════════════════════════════

  struct JointConfig
  {
    std::string name;

    // 【改這裡】你的硬體用什麼定址：DYNAMIXEL 用 id、CAN 用 node id、
    //           EtherCAT 用 slave index
    int device_id{0};

    // URDF 宣告了哪些 interface（用來判斷唯讀關節、跳過不存在的介面）
    std::vector<std::string> command_interfaces;
    std::vector<std::string> state_interfaces;

    // 預先組好的 interface 名稱，避免在即時迴圈裡做字串串接
    std::string position_command_key;
    std::string position_state_key;
    std::string velocity_state_key;

    bool has_command(const std::string & iface) const
    {
      return std::find(command_interfaces.begin(), command_interfaces.end(), iface) !=
             command_interfaces.end();
    }
    bool has_state(const std::string & iface) const
    {
      return std::find(state_interfaces.begin(), state_interfaces.end(), iface) !=
             state_interfaces.end();
    }
  };

  std::vector<JointConfig> joints_;

  // 【改這裡】<hardware> 層級的參數
  std::string port_name_;
  int baud_rate_{1000000};

  // 【改這裡】通訊控制代碼（handle）
  //   序列埠：dynamixel::PortHandler * / PacketHandler *
  //   CAN   ：int socket_fd_
  //   網路  ：你的 client 物件
  //   批次讀寫物件也放這裡：GroupSyncRead / GroupSyncWrite
};

}  // namespace my_hardware

#endif  // MY_HARDWARE__MY_SYSTEM_HPP_
