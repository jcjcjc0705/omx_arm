// ═══════════════════════════════════════════════════════════════════════════
//  ros2_control SystemInterface plugin — 通用骨架（ROS 2 Jazzy）
//
//  這是一份「拿到新機器人就複製過去改」的樣板，不綁任何硬體。
//  所有需要依機器人修改的地方都標了  【改這裡 N】。
//
//  改名步驟（假設新機器人叫 flexgrip）：
//     my_hardware  → flexgrip_hardware      (namespace / package 名)
//     MySystem     → FlexGripSystem         (class 名)
//     my_system    → flexgrip_system        (檔名 / 函式庫名)
//
//  ⚠️ 三個字串必須一致，否則 pluginlib 找不到：
//     xxx.xml 的 <library path=>   ←→  CMakeLists 的 add_library(名字 ...)
//     xxx.xml 的 <class name=>     ←→  URDF 的 <plugin>
//     xxx.xml 的 <class type=>     ←→  下面的 namespace::class
// ═══════════════════════════════════════════════════════════════════════════

#include "my_hardware/my_system.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
rclcpp::Logger hw_logger() { return rclcpp::get_logger("MySystem"); }

// 從 <param> map 取值，沒有就回傳預設值
std::string param_or(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & fallback)
{
  const auto it = params.find(key);
  return (it != params.end()) ? it->second : fallback;
}
}  // namespace

namespace my_hardware
{

// ═══════════════════════════════════════════════════════════════════════════
//  on_init —— 解析 URDF 的 <ros2_control> 區塊
//
//  該做：讀參數、檢查必填項、把字串翻譯成數值存起來
//  不該做：開通訊埠（那是 on_configure 的事）
//
//  這個函式是「字串世界 → 數值世界」的唯一入口。之後的即時迴圈只碰數值，
//  不再查 map、不再轉字串。
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn MySystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // 基底類別會依 URDF 宣告建好所有 state / command interface 的儲存空間
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & info = get_hardware_info();

  // ── 【改這裡 1】<hardware> 層級參數 ──────────────────────────────────
  // 對應 URDF：<hardware><param name="xxx">值</param></hardware>
  // 序列埠機器人：port_name / baud_rate
  // 網路機器人  ：ip / port
  // socket 架構 ：socket_path
  port_name_ = param_or(info.hardware_parameters, "port_name", "/dev/ttyUSB0");
  baud_rate_ = std::stoi(param_or(info.hardware_parameters, "baud_rate", "1000000"));

  // ── 逐關節解析 ───────────────────────────────────────────────────────
  joints_.clear();
  for (const auto & joint : info.joints)
  {
    JointConfig cfg;
    cfg.name = joint.name;

    // 【改這裡 2】你的機器人需要哪些 per-joint 參數
    // 必填的用 find() 檢查、缺了就 return ERROR；選填的用 param_or()
    const auto id_it = joint.parameters.find("device_id");
    if (id_it == joint.parameters.end())
    {
      RCLCPP_FATAL(hw_logger(), "Joint '%s' has no device_id. ", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    cfg.device_id = std::stoi(id_it->second);

    // 記錄這個關節宣告了哪些 interface。
    // 唯讀關節（例如遙操作的 leader）沒有 command_interface，之後迴圈要跳過。
    for (const auto & ci : joint.command_interfaces)
    {
      cfg.command_interfaces.push_back(ci.name);
    }
    for (const auto & si : joint.state_interfaces)
    {
      cfg.state_interfaces.push_back(si.name);
    }

    // 預先組好 interface 名稱字串，避免在即時迴圈裡做字串串接（會 heap 配置）
    cfg.position_command_key = cfg.name + "/" + hardware_interface::HW_IF_POSITION;
    cfg.position_state_key = cfg.name + "/" + hardware_interface::HW_IF_POSITION;
    cfg.velocity_state_key = cfg.name + "/" + hardware_interface::HW_IF_VELOCITY;

    joints_.push_back(cfg);
  }

  // 【改這裡 3】如果 URDF 有 <sensor>（TOF、IMU、力覺…），在這裡解析
  // for (const auto & sensor : info.sensors) { ... }

  RCLCPP_INFO(hw_logger(), "Initialized %zu joints.", joints_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_configure —— 建立與硬體的連線
//
//  該做：開埠 / 連線、掃描裝置、寫入不常變的設定（EEPROM）
//  不該做：開扭力（那是 on_activate）
//
//  這裡失敗要 return ERROR，controller_manager 會停在 unconfigured 狀態。
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn MySystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // ── 【改這裡 4】開啟通訊 ─────────────────────────────────────────────
  // TODO:
  //   1. 開埠 / 建立連線，失敗 → return ERROR
  //   2. 對每個裝置 PING，確認都在線上
  //   3. 寫入 operating_mode、PID、limit 等一次性設定
  //
  // 序列埠範例：
  //   port_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
  //   if (!port_->openPort()) { RCLCPP_FATAL(...); return ERROR; }
  //   if (!port_->setBaudRate(baud_rate_)) { ... return ERROR; }

  // 狀態歸零，讓整條鏈路在還沒接硬體時也能跑
  for (const auto & j : joints_)
  {
    set_state(j.position_state_key, 0.0);
    set_state(j.velocity_state_key, 0.0);
  }

  RCLCPP_INFO(hw_logger(), "configure completed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_activate —— 開始接受指令
//
//  ⚠️ 安全關鍵：一定要先把「指令」設成「目前實際位置」，
//     否則啟用瞬間手臂會從現況跳到上一次的舊指令（常常是 0），全軸同時全速動。
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn MySystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 【改這裡 5】開扭力 / 進 servo 模式
  // TODO: for each device → torque enable

  for (const auto & j : joints_)
  {
    if (j.has_command(hardware_interface::HW_IF_POSITION))
    {
      set_command(j.position_command_key, get_state<double>(j.position_state_key));
    }
  }

  RCLCPP_INFO(hw_logger(), "activate completed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_deactivate —— 安全停機
//  該做：關扭力，讓機構洩力
//  不該做：關埠（那是 on_cleanup / on_shutdown）
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn MySystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 【改這裡 6】關扭力
  // TODO: for each device → torque disable

  RCLCPP_INFO(hw_logger(), "deactivate completed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  read —— 每個控制週期開頭：硬體回授 → set_state()
//
//  ⚠️ 即時迴圈內：禁止 malloc、字串串接、印大量 log、阻塞式等待
//  ⚠️ 多裝置一定要批次讀（sync read），逐顆讀會嚴重限制可達頻率
//  ⚠️ 單位換算在這裡做完 —— 對外一律 SI（rad, rad/s, N·m）
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::return_type MySystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // ── 【改這裡 7】真正的讀取 ───────────────────────────────────────────
  // TODO:
  //   1. 一次批次讀回所有裝置的原始值
  //   2. 換算成 SI 單位
  //   3. set_state(key, value)
  //   4. 通訊失敗 → return hardware_interface::return_type::ERROR
  //
  // 範例：
  //   sync_read_.txRxPacket();
  //   for (auto & j : joints_) {
  //     int32_t tick = sync_read_.getData(j.device_id, ADDR_PRESENT_POSITION, 4);
  //     set_state(j.position_state_key, tick_to_rad(tick));
  //   }

  // ── 以下是「沒有硬體時的替身行為」：指令直接複製成狀態 ────────────────
  //    接上真硬體後整段刪掉
  const double dt = period.seconds();
  for (const auto & j : joints_)
  {
    if (!j.has_command(hardware_interface::HW_IF_POSITION))
    {
      continue;  // 唯讀關節沒有指令可以複製
    }
    const double cmd = get_command<double>(j.position_command_key);
    if (std::isnan(cmd))
    {
      continue;  // controller 還沒寫過任何指令
    }
    const double prev = get_state<double>(j.position_state_key);
    set_state(j.position_state_key, cmd);
    set_state(j.velocity_state_key, dt > 0.0 ? (cmd - prev) / dt : 0.0);
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  write —— 每個控制週期結尾：get_command() → 送給硬體
//
//  同樣的即時性限制。多裝置用批次寫（sync write）。
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::return_type MySystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // ── 【改這裡 8】真正的寫入 ───────────────────────────────────────────
  // TODO:
  //   1. for each joint → get_command(key)
  //   2. NaN 要跳過
  //   3. 換算成硬體原始單位
  //   4. 批次送出
  //   5. 通訊失敗 → return ERROR
  //
  // 範例：
  //   for (auto & j : joints_) {
  //     if (!j.has_command(HW_IF_POSITION)) continue;
  //     double rad = get_command<double>(j.position_command_key);
  //     if (std::isnan(rad)) continue;
  //     sync_write_.addParam(j.device_id, rad_to_tick(rad));
  //   }
  //   sync_write_.txPacket();

  return hardware_interface::return_type::OK;
}

}  // namespace my_hardware

// 讓 pluginlib 能在執行期找到這個 class
PLUGINLIB_EXPORT_CLASS(my_hardware::MySystem, hardware_interface::SystemInterface)
