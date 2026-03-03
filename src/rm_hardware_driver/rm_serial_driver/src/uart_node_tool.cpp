#include "rm_serial_driver/uart_node_tool.hpp"
#include "rm_serial_driver/uart_node.hpp"
#include "rm_serial_driver/uart_protocol.hpp"
#include "rm_interfaces/msg/serial_send_data.hpp"

#include <thread>
#include <chrono>

// pkaLoggerCenter
#include "rm_utils/pkaLoggerCenter.hpp"

namespace pka {
namespace serial_driver {

// ─────────────────────────────────────────────────────────────────────────────
//  话题回调 —— 仅缓存，不触发发送
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::gimbal_cmd_callback(
  UARTNode * node,
  const rm_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  node->cached_fire_advice_ = msg->fire_advice;
  node->cached_pitch_       = static_cast<float>(msg->pitch);
  node->cached_yaw_         = static_cast<float>(msg->yaw);
  node->cached_distance_    = static_cast<float>(msg->distance);

  if (node->debug_) {
    PKA_DEBUG("serial_node",
      "[sub/gimbal_cmd] fire={} pitch={:.4f} yaw={:.4f} dist={:.4f}",
      msg->fire_advice, msg->pitch, msg->yaw, msg->distance);
  }
}

void UARTNodeTool::cmd_vel_callback(
  UARTNode * node,
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const float scale = static_cast<float>(node->cmd_vel_linear_scale_);
  node->cached_linear_x_  = static_cast<float>(msg->linear.x)  * scale;
  node->cached_linear_y_  = static_cast<float>(msg->linear.y)  * scale;
  node->cached_angular_z_ = static_cast<float>(msg->angular.z);

  if (node->debug_) {
    PKA_DEBUG("serial_node",
      "[sub/cmd_vel] vx={:.4f} vy={:.4f} wz={:.4f}  (scale={})",
      node->cached_linear_x_, node->cached_linear_y_,
      node->cached_angular_z_, scale);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  串口发送辅助
//
//  严格参照 serial_example.cc：
//    size_t bytes_wrote = my_serial.write(test_string);
//
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::do_send(
  UARTNode * node,
  const rm_interfaces::msg::SerialSendData & data)
{
  if (!node->serial_ || !node->serial_->isOpen()) {
    on_error(node, SerialErrorCode::DEVICE_NOT_FOUND, "do_send: port not open");
    return;
  }

  try {
    // 打包为 std::string（32字节）
    std::string packed = UARTProtocol::pack_serial_send_data(data);

    // size_t bytes_wrote = my_serial.write(test_string);
    size_t bytes_wrote = node->serial_->write(packed);

    if (bytes_wrote != packed.size()) {
      std::string err = "Partial write: wrote " + std::to_string(bytes_wrote)
                      + " / " + std::to_string(packed.size()) + " bytes";
      PKA_WARN("serial_node", "[send] {}", err);
      on_error(node, SerialErrorCode::TIMEOUT, err);
      return;
    }

    node->consecutive_failure_count_ = 0;
    node->is_healthy_                = true;
    node->last_success_time_         = node->now();

    if (node->debug_) {
      PKA_DEBUG("serial_node",
        "[send] bytes_wrote={}/{} fire={} pitch={:.4f} yaw={:.4f} dist={:.4f} "
        "vx={:.4f} vy={:.4f} wz={:.4f}",
        bytes_wrote, packed.size(),
        data.fire_advice, data.pitch, data.yaw, data.distance,
        data.linear_x, data.linear_y, data.angular_z);
      PKA_DEBUG("serial_node",
        "[send raw] {}", UARTProtocol::format_hex(packed));
    }

  } catch (const serial::PortNotOpenedException & e) {
    PKA_ERROR("serial_node", "[send] PortNotOpened: {}", e.what());
    on_error(node, SerialErrorCode::DEVICE_NOT_FOUND, e.what());
  } catch (const serial::SerialException & e) {
    PKA_ERROR("serial_node", "[send] SerialException: {}", e.what());
    on_error(node, SerialErrorCode::UNKNOWN_ERROR, e.what());
  } catch (const serial::IOException & e) {
    PKA_ERROR("serial_node", "[send] IOException: {}", e.what());
    on_error(node, SerialErrorCode::HARDWARE_ERROR, e.what());
  } catch (const std::exception & e) {
    PKA_ERROR("serial_node", "[send] Exception: {}", e.what());
    on_error(node, SerialErrorCode::UNKNOWN_ERROR, e.what());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  mode=0 模拟接收定时器
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::mock_recv_timer_cb(UARTNode * node)
{
  rm_interfaces::msg::SerialReceiveData msg;
  msg.header.stamp           = node->now();
  msg.header.frame_id        = "mock_serial_receive";
  msg.mode                   = node->mock_mode_;
  msg.roll                   = node->mock_roll_;
  msg.pitch                  = node->mock_pitch_;
  msg.yaw                    = node->mock_yaw_;
  msg.chassis_imu_yaw_offset = node->mock_chassis_imu_yaw_offset_;

  node->recv_pub_->publish(msg);
  node->broadcastGimbalTF(msg);

  if (node->debug_) {
    PKA_DEBUG("serial_node",
      "[mock_recv] mode={} roll={:.3f}° pitch={:.3f}° yaw={:.3f}° yaw_off={:.3f}°",
      msg.mode, msg.roll, msg.pitch, msg.yaw, msg.chassis_imu_yaw_offset);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  mode=1 定时发送缓存数据
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::send_timer_cb(UARTNode * node)
{
  rm_interfaces::msg::SerialSendData data;
  data.fire_advice = node->cached_fire_advice_;
  data.pitch       = node->cached_pitch_;
  data.yaw         = node->cached_yaw_;
  data.distance    = node->cached_distance_;
  data.linear_x    = node->cached_linear_x_;
  data.linear_y    = node->cached_linear_y_;
  data.angular_z   = node->cached_angular_z_;
  do_send(node, data);
}

// ─────────────────────────────────────────────────────────────────────────────
//  mode=1,2 接收定时器
//
//  严格参照 serial_example.cc：
//    string result = my_serial.read(N);    // read(size_t) → std::string
//
//  帧同步策略（修复版）：
//
//  【问题根源】
//    串口是字节流，缓冲区里可能存有上一帧的残余数据，不能直接 read(32)。
//    必须先找到帧头 0xFF 才能保证字节对齐。
//
//  【旧版 bug】
//    while 条件写成 available() >= pkt(32)，每次 read(1) 消耗一字节后
//    available() 递减，当剩余字节数跌破 32 时循环提前退出，
//    已消耗掉的字节永久丢失，造成帧头找到一半就放弃的问题。
//
//  【修复方案】
//    第一步：找帧头时只要 available() >= 1 就继续，
//            不关心此时剩余字节总数够不够一帧。
//    第二步：找到帧头后，单独判断剩余字节 >= pkt-1，
//            不够则本次放弃（帧头字节已消耗，下次会重新找）。
//            由于下位机持续发送，很快会有新的完整帧到达。
//
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::recv_timer_cb(UARTNode * node)
{
  if (!node->serial_ || !node->serial_->isOpen()) { return; }

  try {
    const size_t pkt = UARTProtocol::RECV_PACKET_SIZE;  // 32

    // ── 第一步：逐字节找帧头 ──────────────────────────────────────────────────
    // 只要缓冲区有数据就继续找，条件是 available() >= 1，
    // 而不是 available() >= pkt（旧版 bug 所在）。
    std::string hdr;
    while (node->serial_->available() >= 1) {
      // string result = my_serial.read(1);
      hdr = node->serial_->read(1);
      if (hdr.empty()) { return; }
      if (static_cast<uint8_t>(hdr[0]) == UARTProtocol::FRAME_HEADER) { break; }
    }

    // while 可能因 available() 归零而退出（没找到帧头），也可能正常 break（找到了）
    if (hdr.empty() || static_cast<uint8_t>(hdr[0]) != UARTProtocol::FRAME_HEADER) {
      // 缓冲区里暂时没有帧头，等下次定时器再扫
      return;
    }

    // ── 第二步：确认剩余字节足够读完整帧，不够则放弃本次 ────────────────────
    // 帧头已消耗 1 字节，还需要 pkt-1 = 31 字节
    if (node->serial_->available() < pkt - 1) {
      if (node->debug_) {
        PKA_DEBUG("serial_node",
          "[recv] Header found but only {}/{} remaining bytes, waiting next tick",
          node->serial_->available(), pkt - 1);
      }
      // 本次放弃；帧头字节已丢弃，下次定时器会重新找新帧头。
      // 下位机持续发帧，下次定时器触发时新的完整帧通常已到达。
      return;
    }

    // ── 第三步：读剩余 pkt-1 字节 ────────────────────────────────────────────
    // string result = my_serial.read(pkt - 1);
    std::string rest = node->serial_->read(pkt - 1);

    if (rest.size() != pkt - 1) {
      std::string err = "Incomplete packet: got "
                      + std::to_string(rest.size() + 1)
                      + "/" + std::to_string(pkt) + " bytes";
      PKA_WARN("serial_node", "[recv] {}", err);
      on_error(node, SerialErrorCode::FRAME_ERROR, err);
      return;
    }

    // ── 第四步：拼合完整帧并解包 ─────────────────────────────────────────────
    std::string raw = hdr + rest;   // 32 字节完整帧

    rm_interfaces::msg::SerialReceiveData recv_msg;
    if (!UARTProtocol::unpack_serial_data(raw, recv_msg)) {
      PKA_WARN("serial_node",
        "[recv] Frame parse failed  raw={}", UARTProtocol::format_hex(raw));
      on_error(node, SerialErrorCode::FRAME_ERROR, "Frame parse failed");
      return;
    }

    // ── 第五步：发布 & 广播 TF ────────────────────────────────────────────────
    recv_msg.header.stamp    = node->now();
    recv_msg.header.frame_id = "serial_receive";
    node->recv_pub_->publish(recv_msg);
    node->broadcastGimbalTF(recv_msg);

    node->consecutive_failure_count_ = 0;
    node->is_healthy_                = true;
    node->last_success_time_         = node->now();

    if (node->debug_) {
      PKA_DEBUG("serial_node",
        "[recv] bytes_read={}/{} mode={} roll={:.3f}° pitch={:.3f}° "
        "yaw={:.3f}° yaw_off={:.3f}°",
        raw.size(), pkt,
        recv_msg.mode, recv_msg.roll, recv_msg.pitch,
        recv_msg.yaw, recv_msg.chassis_imu_yaw_offset);
      PKA_DEBUG("serial_node",
        "[recv raw] {}", UARTProtocol::format_hex(raw));
    }

  } catch (const serial::PortNotOpenedException & e) {
    PKA_ERROR("serial_node", "[recv] PortNotOpened: {}", e.what());
    on_error(node, SerialErrorCode::DEVICE_NOT_FOUND, e.what());
  } catch (const serial::SerialException & e) {
    PKA_ERROR("serial_node", "[recv] SerialException: {}", e.what());
    on_error(node, SerialErrorCode::UNKNOWN_ERROR, e.what());
  } catch (const serial::IOException & e) {
    PKA_ERROR("serial_node", "[recv] IOException: {}", e.what());
    on_error(node, SerialErrorCode::HARDWARE_ERROR, e.what());
  } catch (const std::exception & e) {
    PKA_ERROR("serial_node", "[recv] Exception: {}", e.what());
    on_error(node, SerialErrorCode::UNKNOWN_ERROR, e.what());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  mode=2 虚拟发送定时器
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::virtual_send_timer_cb(UARTNode * node)
{
  rm_interfaces::msg::SerialSendData vd;
  vd.fire_advice = node->virtual_fire_advice_;
  vd.pitch       = node->virtual_pitch_;
  vd.yaw         = node->virtual_yaw_;
  vd.distance    = node->virtual_distance_;
  // virtual_linear_x/y 同样乘缩放系数，与 mode=1 语义一致
  vd.linear_x    = node->virtual_linear_x_ * static_cast<float>(node->cmd_vel_linear_scale_);
  vd.linear_y    = node->virtual_linear_y_ * static_cast<float>(node->cmd_vel_linear_scale_);
  vd.angular_z   = node->virtual_angular_z_;

  if (node->debug_) {
    PKA_DEBUG("serial_node",
      "[virtual_send] fire={} pitch={:.4f} yaw={:.4f} dist={:.4f} "
      "vx={:.4f} vy={:.4f} wz={:.4f}",
      vd.fire_advice, vd.pitch, vd.yaw, vd.distance,
      vd.linear_x, vd.linear_y, vd.angular_z);
  }

  do_send(node, vd);
}

// ─────────────────────────────────────────────────────────────────────────────
//  健康检查定时器
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::health_timer_cb(UARTNode * node)
{
  if (node->serial_mode_ == 0 || !node->enable_auto_restart_) { return; }

  if (node->max_restart_attempts_ > 0 &&
      node->total_restart_attempts_ >= node->max_restart_attempts_)
  {
    PKA_FATAL("serial_node",
      "Max restart attempts ({}) reached, manual intervention required",
      node->max_restart_attempts_);
    return;
  }

  bool port_ok      = node->serial_ && node->serial_->isOpen();
  bool need_restart = !port_ok
    || !node->is_healthy_
    || (node->consecutive_failure_count_ >= node->max_failure_count_);

  if (!need_restart) {
    if (node->debug_) {
      PKA_DEBUG("serial_node",
        "[health] OK  port_open={} consecutive_failures={}",
        port_ok, node->consecutive_failure_count_);
    }
    return;
  }

  // 冷却期检查
  if (node->last_restart_time_.nanoseconds() > 0) {
    double elapsed = (node->now() - node->last_restart_time_).seconds();
    if (elapsed < node->restart_cooldown_) {
      PKA_WARN("serial_node",
        "[health] Unhealthy but in cooldown, remaining={:.1f}s",
        node->restart_cooldown_ - elapsed);
      return;
    }
  }

  PKA_WARN("serial_node",
    "[health] Unhealthy: port_open={} is_healthy={} failures={}/{} → restart #{}/{}",
    port_ok, node->is_healthy_,
    node->consecutive_failure_count_, node->max_failure_count_,
    node->total_restart_attempts_ + 1, node->max_restart_attempts_);

  if (restart_serial(node)) {
    PKA_INFO("serial_node",
      "[health] Restart succeeded (attempt #{})", node->total_restart_attempts_);
  } else {
    PKA_ERROR("serial_node",
      "[health] Restart failed (attempt #{})", node->total_restart_attempts_);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  串口重启
// ─────────────────────────────────────────────────────────────────────────────

bool UARTNodeTool::restart_serial(UARTNode * node)
{
  if (!node->enable_auto_restart_) { return false; }

  node->last_restart_time_ = node->now();
  node->total_restart_attempts_++;

  // 根据错误类型调整等待时长
  int delay_ms = node->restart_delay_;
  switch (node->last_error_code_) {
    case SerialErrorCode::PERMISSION_DENIED:
    case SerialErrorCode::DEVICE_NOT_FOUND:
      delay_ms = node->restart_delay_ * 2;
      break;
    case SerialErrorCode::TIMEOUT:
    case SerialErrorCode::FRAME_ERROR:
      delay_ms = std::max(node->restart_delay_ / 2, 100);
      break;
    default:
      break;
  }

  PKA_INFO("serial_node",
    "[restart] Waiting {}ms before reopen (error={})",
    delay_ms, getErrorString(node->last_error_code_));

  std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

  // 重新调用 init_serial()（会先关旧对象再新建）
  node->init_serial();

  bool ok = node->serial_ && node->serial_->isOpen() && node->is_healthy_;
  if (ok) { node->consecutive_failure_count_ = 0; }
  return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
//  错误处理
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::on_error(
  UARTNode * node,
  SerialErrorCode code,
  const std::string & msg)
{
  node->consecutive_failure_count_++;
  node->is_healthy_      = false;
  node->last_error_code_ = code;
  node->last_error_msg_  = msg;

  // TIMEOUT / FRAME_ERROR 降级到 WARN，避免日志刷屏
  if (code == SerialErrorCode::TIMEOUT || code == SerialErrorCode::FRAME_ERROR) {
    PKA_WARN("serial_node",
      "[error] {} | {} (consecutive={})",
      getErrorString(code), msg, node->consecutive_failure_count_);
  } else {
    PKA_ERROR("serial_node",
      "[error] {} | {} (consecutive={})",
      getErrorString(code), msg, node->consecutive_failure_count_);
  }
}

}  // namespace serial_driver
}  // namespace pka