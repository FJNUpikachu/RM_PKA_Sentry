#include "rm_serial_driver/uart_node_tool.hpp"
#include "rm_serial_driver/uart_node.hpp"

// 协议静态方法
#include "rm_serial_driver/protocol/sentry_protocol.hpp"
#include "rm_serial_driver/protocol/infantry_protocol.hpp"
#include "rm_serial_driver/protocol/uart_protocol_base.hpp"

#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"

#include <thread>
#include <chrono>

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
      "[sub/cmd_vel] vx={:.4f} vy={:.4f} wz={:.4f} (scale={})",
      node->cached_linear_x_, node->cached_linear_y_,
      node->cached_angular_z_, scale);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SetMode 服务通知辅助
//
//  只在 mode 值发生变化时才发起调用，避免每帧都发请求。
//  使用异步 async_send_request，回调仅做日志记录，不阻塞接收定时器。
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::notify_set_mode(UARTNode * node, uint8_t mode)
{
  // 没有变化则跳过
  if (mode == node->last_sent_mode_) { return; }

  // 服务端尚未就绪时不阻塞，仅打印警告并跳过（下一帧会重试）
  if (!node->set_mode_client_->service_is_ready()) {
    PKA_WARN("serial_node",
      "[set_mode] Service '{}' not ready, skipping mode={} (will retry on next change)",
      node->set_mode_service_name_, mode);
    return;
  }

  // 更新缓存（先更新，防止高频帧在响应返回前重复发送）
  node->last_sent_mode_ = mode;

  auto request = std::make_shared<rm_interfaces::srv::SetMode::Request>();
  request->mode = mode;

  // 异步发送，回调仅记录日志
  node->set_mode_client_->async_send_request(
    request,
    [node, mode](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture future) {
      auto resp = future.get();
      if (resp->success) {
        PKA_INFO("serial_node",
          "[set_mode] armor_detector mode set to {} ({})",
          mode, mode == 0 ? "AUTO_AIM_RED" : (mode == 1 ? "AUTO_AIM_BLUE" : "OTHER"));
      } else {
        // 失败时将 last_sent_mode_ 复位，以便下次收到同一 mode 时重试
        node->last_sent_mode_ = 255;
        PKA_ERROR("serial_node",
          "[set_mode] armor_detector rejected mode={}, msg={}",
          mode, resp->message);
      }
    });

  if (node->debug_) {
    PKA_DEBUG("serial_node",
      "[set_mode] Async request sent: mode={}", mode);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  串口发送辅助 —— 哨兵
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::do_send_sentry(
  UARTNode * node,
  const rm_interfaces::msg::SentrySerialSendData & data)
{
  if (!node->serial_ || !node->serial_->isOpen()) {
    on_error(node, SerialErrorCode::DEVICE_NOT_FOUND, "do_send_sentry: port not open");
    return;
  }

  try {
    std::string packed = SentryProtocol::pack(data);
    size_t bytes_wrote = node->serial_->write(packed);

    if (bytes_wrote != packed.size()) {
      std::string err = "Partial write: " + std::to_string(bytes_wrote)
                      + "/" + std::to_string(packed.size());
      PKA_WARN("serial_node", "[sentry send] {}", err);
      on_error(node, SerialErrorCode::TIMEOUT, err);
      return;
    }

    node->consecutive_failure_count_ = 0;
    node->is_healthy_                = true;
    node->last_success_time_         = node->now();

    if (node->debug_) {
      PKA_DEBUG("serial_node",
        "[sentry send] bytes={}/{} fire={} pitch={:.4f} yaw={:.4f} dist={:.4f} "
        "vx={:.4f} vy={:.4f} wz={:.4f}",
        bytes_wrote, packed.size(),
        data.fire_advice, data.pitch, data.yaw, data.distance,
        data.linear_x, data.linear_y, data.angular_z);
      PKA_DEBUG("serial_node", "[sentry send raw] {}", IUARTProtocol::format_hex(packed));
    }

  } catch (const serial::PortNotOpenedException & e) {
    PKA_ERROR("serial_node", "[sentry send] PortNotOpened: {}", e.what());
    on_error(node, SerialErrorCode::DEVICE_NOT_FOUND, e.what());
  } catch (const serial::SerialException & e) {
    PKA_ERROR("serial_node", "[sentry send] SerialException: {}", e.what());
    on_error(node, SerialErrorCode::UNKNOWN_ERROR, e.what());
  } catch (const serial::IOException & e) {
    PKA_ERROR("serial_node", "[sentry send] IOException: {}", e.what());
    on_error(node, SerialErrorCode::HARDWARE_ERROR, e.what());
  } catch (const std::exception & e) {
    PKA_ERROR("serial_node", "[sentry send] Exception: {}", e.what());
    on_error(node, SerialErrorCode::UNKNOWN_ERROR, e.what());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  串口发送辅助 —— 步兵
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::do_send_infantry(
  UARTNode * node,
  const rm_interfaces::msg::InfantrySerialSendData & data)
{
  if (!node->serial_ || !node->serial_->isOpen()) {
    on_error(node, SerialErrorCode::DEVICE_NOT_FOUND, "do_send_infantry: port not open");
    return;
  }

  try {
    std::string packed = InfantryProtocol::pack(data);
    size_t bytes_wrote = node->serial_->write(packed);

    if (bytes_wrote != packed.size()) {
      std::string err = "Partial write: " + std::to_string(bytes_wrote)
                      + "/" + std::to_string(packed.size());
      PKA_WARN("serial_node", "[infantry send] {}", err);
      on_error(node, SerialErrorCode::TIMEOUT, err);
      return;
    }

    node->consecutive_failure_count_ = 0;
    node->is_healthy_                = true;
    node->last_success_time_         = node->now();

    if (node->debug_) {
      PKA_DEBUG("serial_node",
        "[infantry send] bytes={}/{} fire={} pitch={:.4f} yaw={:.4f} dist={:.4f}",
        bytes_wrote, packed.size(),
        data.fire_advice, data.pitch, data.yaw, data.distance);
      PKA_DEBUG("serial_node", "[infantry send raw] {}", IUARTProtocol::format_hex(packed));
    }

  } catch (const serial::PortNotOpenedException & e) {
    PKA_ERROR("serial_node", "[infantry send] PortNotOpened: {}", e.what());
    on_error(node, SerialErrorCode::DEVICE_NOT_FOUND, e.what());
  } catch (const serial::SerialException & e) {
    PKA_ERROR("serial_node", "[infantry send] SerialException: {}", e.what());
    on_error(node, SerialErrorCode::UNKNOWN_ERROR, e.what());
  } catch (const serial::IOException & e) {
    PKA_ERROR("serial_node", "[infantry send] IOException: {}", e.what());
    on_error(node, SerialErrorCode::HARDWARE_ERROR, e.what());
  } catch (const std::exception & e) {
    PKA_ERROR("serial_node", "[infantry send] Exception: {}", e.what());
    on_error(node, SerialErrorCode::UNKNOWN_ERROR, e.what());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  裁判系统消息发布辅助（仅哨兵）
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::publish_judge_msgs(
  UARTNode * node,
  const rm_interfaces::msg::SentrySerialReceiveData & recv_msg)
{
  {
    rm_interfaces::msg::GameStatus gs;
    gs.game_progress = recv_msg.game_progress;
    node->game_status_pub_->publish(gs);
  }
  {
    rm_interfaces::msg::RfidStatus rs;
    rs.friendly_supply_zone_non_exchange =
      recv_msg.friendly_supply_zone_non_exchange;
    node->rfid_status_pub_->publish(rs);
  }
  {
    rm_interfaces::msg::RobotStatus rbs;
    rbs.current_hp = recv_msg.current_hp;
    node->robot_status_pub_->publish(rbs);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  mode=0 模拟接收定时器
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::mock_recv_timer_cb(UARTNode * node)
{
  switch (node->robot_type_) {

    // ── 哨兵 mock ───────────────────────────────────────────────────────────
    case RobotType::SENTRY: {
      rm_interfaces::msg::SentrySerialReceiveData msg;
      msg.header.stamp           = node->now();
      msg.header.frame_id        = "mock_serial_receive";
      msg.mode                   = node->mock_mode_;
      msg.roll                   = node->mock_roll_;
      msg.pitch                  = node->mock_pitch_;
      msg.yaw                    = node->mock_yaw_;
      msg.chassis_imu_yaw_offset = node->mock_chassis_imu_yaw_offset_;
      msg.game_progress                    = 0;
      msg.friendly_supply_zone_non_exchange = 0;
      msg.current_hp                        = 0;

      node->sentry_recv_pub_->publish(msg);
      node->broadcastGimbalTF(msg.roll, msg.pitch, msg.yaw,
                               msg.chassis_imu_yaw_offset);
      publish_judge_msgs(node, msg);

      // ── 通知 armor_detector 切换模式（仅当 mode 变化时）──────────────────
      notify_set_mode(node, msg.mode);

      if (node->debug_) {
        PKA_DEBUG("serial_node",
          "[mock/sentry] mode={} roll={:.3f}° pitch={:.3f}° yaw={:.3f}°",
          msg.mode, msg.roll, msg.pitch, msg.yaw);
      }
      break;
    }

    // ── 步兵 mock ───────────────────────────────────────────────────────────
    case RobotType::INFANTRY: {
      rm_interfaces::msg::InfantrySerialReceiveData msg;
      msg.header.stamp    = node->now();
      msg.header.frame_id = "mock_serial_receive";
      msg.mode            = node->mock_mode_;
      msg.roll            = node->mock_roll_;
      msg.pitch           = node->mock_pitch_;
      msg.yaw             = node->mock_yaw_;

      node->infantry_recv_pub_->publish(msg);
      node->broadcastGimbalTF(msg.roll, msg.pitch, msg.yaw);

      // ── 通知 armor_detector 切换模式（仅当 mode 变化时）──────────────────
      notify_set_mode(node, msg.mode);

      if (node->debug_) {
        PKA_DEBUG("serial_node",
          "[mock/infantry] mode={} roll={:.3f}° pitch={:.3f}° yaw={:.3f}°",
          msg.mode, msg.roll, msg.pitch, msg.yaw);
      }
      break;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  mode=1 定时发送缓存数据
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::send_timer_cb(UARTNode * node)
{
  switch (node->robot_type_) {

    case RobotType::SENTRY: {
      rm_interfaces::msg::SentrySerialSendData data;
      data.fire_advice = node->cached_fire_advice_;
      data.pitch       = node->cached_pitch_;
      data.yaw         = node->cached_yaw_;
      data.distance    = node->cached_distance_;
      data.linear_x    = node->cached_linear_x_;
      data.linear_y    = node->cached_linear_y_;
      data.angular_z   = node->cached_angular_z_;
      do_send_sentry(node, data);
      break;
    }

    case RobotType::INFANTRY: {
      rm_interfaces::msg::InfantrySerialSendData data;
      data.fire_advice = node->cached_fire_advice_;
      data.pitch       = node->cached_pitch_;
      data.yaw         = node->cached_yaw_;
      data.distance    = node->cached_distance_;
      do_send_infantry(node, data);
      break;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  mode=1,2 接收定时器
// ─────────────────────────────────────────────────────────────────────────────

void UARTNodeTool::recv_timer_cb(UARTNode * node)
{
  if (!node->serial_ || !node->serial_->isOpen()) { return; }

  // 协议对象提供正确的包长
  const size_t pkt = node->protocol_->recv_packet_size();

  try {
    while (node->serial_->available() >= pkt) {

      std::string buf = node->serial_->read(pkt);
      if (buf.size() != pkt) { break; }

      // ── 帧头搜索（通用，利用虚接口的 frame_header()） ────────────────────
      const uint8_t hdr = node->protocol_->frame_header();
      if (static_cast<uint8_t>(buf[0]) != hdr) {
        size_t found = std::string::npos;
        for (size_t i = 1; i < buf.size(); ++i) {
          if (static_cast<uint8_t>(buf[i]) == hdr) { found = i; break; }
        }
        if (found == std::string::npos) { continue; }
        buf = buf.substr(found);
        if (node->serial_->available() < found) { break; }
        buf += node->serial_->read(found);
        if (buf.size() != pkt) { break; }
      }

      // ── 按机器人类型分派解包 ─────────────────────────────────────────────
      const rclcpp::Time stamp_now = node->now();

      switch (node->robot_type_) {

        // ── 哨兵解包 ─────────────────────────────────────────────────────
        case RobotType::SENTRY: {
          rm_interfaces::msg::SentrySerialReceiveData recv_msg;
          if (!SentryProtocol::unpack(buf, recv_msg)) {
            PKA_WARN("serial_node", "[recv/sentry] Unpack failed raw=[{}]",
              IUARTProtocol::format_hex(buf));
            on_error(node, SerialErrorCode::FRAME_ERROR, "sentry unpack failed");
            continue;
          }
          recv_msg.header.stamp = stamp_now +
            rclcpp::Duration::from_seconds(node->timestamp_offset_);
          recv_msg.header.frame_id = "serial_receive";

          node->sentry_recv_pub_->publish(recv_msg);
          node->broadcastGimbalTF(recv_msg.roll, recv_msg.pitch,
                                   recv_msg.yaw, recv_msg.chassis_imu_yaw_offset);
          publish_judge_msgs(node, recv_msg);

          // ── 通知 armor_detector 切换模式（仅当 mode 变化时）──────────────
          notify_set_mode(node, recv_msg.mode);

          node->consecutive_failure_count_ = 0;
          node->is_healthy_                = true;
          node->last_success_time_         = stamp_now;

          if (node->debug_) {
            PKA_DEBUG("serial_node",
              "[recv/sentry] mode={} roll={:.3f}° pitch={:.3f}° yaw={:.3f}° "
              "yaw_off={:.3f}° game_prog={} supply={} hp={}",
              recv_msg.mode, recv_msg.roll, recv_msg.pitch, recv_msg.yaw,
              recv_msg.chassis_imu_yaw_offset,
              recv_msg.game_progress,
              recv_msg.friendly_supply_zone_non_exchange,
              recv_msg.current_hp);
            PKA_DEBUG("serial_node", "[recv/sentry raw] {}", IUARTProtocol::format_hex(buf));
          }
          break;
        }

        // ── 步兵解包 ─────────────────────────────────────────────────────
        case RobotType::INFANTRY: {
          rm_interfaces::msg::InfantrySerialReceiveData recv_msg;
          if (!InfantryProtocol::unpack(buf, recv_msg)) {
            PKA_WARN("serial_node", "[recv/infantry] Unpack failed raw=[{}]",
              IUARTProtocol::format_hex(buf));
            on_error(node, SerialErrorCode::FRAME_ERROR, "infantry unpack failed");
            continue;
          }
          recv_msg.header.stamp = stamp_now +
            rclcpp::Duration::from_seconds(node->timestamp_offset_);
          recv_msg.header.frame_id = "serial_receive";

          node->infantry_recv_pub_->publish(recv_msg);
          node->broadcastGimbalTF(recv_msg.roll, recv_msg.pitch, recv_msg.yaw);

          // ── 通知 armor_detector 切换模式（仅当 mode 变化时）──────────────
          notify_set_mode(node, recv_msg.mode);

          node->consecutive_failure_count_ = 0;
          node->is_healthy_                = true;
          node->last_success_time_         = stamp_now;

          if (node->debug_) {
            PKA_DEBUG("serial_node",
              "[recv/infantry] mode={} roll={:.3f}° pitch={:.3f}° yaw={:.3f}°",
              recv_msg.mode, recv_msg.roll, recv_msg.pitch, recv_msg.yaw);
            PKA_DEBUG("serial_node",
              "[recv/infantry raw] {}", IUARTProtocol::format_hex(buf));
          }
          break;
        }
      }
    }  // while

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
  switch (node->robot_type_) {

    case RobotType::SENTRY: {
      rm_interfaces::msg::SentrySerialSendData vd;
      vd.fire_advice = node->virtual_fire_advice_;
      vd.pitch       = node->virtual_pitch_;
      vd.yaw         = node->virtual_yaw_;
      vd.distance    = node->virtual_distance_;
      vd.linear_x    = node->virtual_linear_x_ *
                        static_cast<float>(node->cmd_vel_linear_scale_);
      vd.linear_y    = node->virtual_linear_y_ *
                        static_cast<float>(node->cmd_vel_linear_scale_);
      vd.angular_z   = node->virtual_angular_z_;

      if (node->debug_) {
        PKA_DEBUG("serial_node",
          "[virtual_send/sentry] fire={} pitch={:.4f} yaw={:.4f} dist={:.4f} "
          "vx={:.4f} vy={:.4f} wz={:.4f}",
          vd.fire_advice, vd.pitch, vd.yaw, vd.distance,
          vd.linear_x, vd.linear_y, vd.angular_z);
      }
      do_send_sentry(node, vd);
      break;
    }

    case RobotType::INFANTRY: {
      rm_interfaces::msg::InfantrySerialSendData vd;
      vd.fire_advice = node->virtual_fire_advice_;
      vd.pitch       = node->virtual_pitch_;
      vd.yaw         = node->virtual_yaw_;
      vd.distance    = node->virtual_distance_;

      if (node->debug_) {
        PKA_DEBUG("serial_node",
          "[virtual_send/infantry] fire={} pitch={:.4f} yaw={:.4f} dist={:.4f}",
          vd.fire_advice, vd.pitch, vd.yaw, vd.distance);
      }
      do_send_infantry(node, vd);
      break;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  健康检查定时器（协议无关，逻辑不变）
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
        "[health] OK port_open={} consecutive_failures={}",
        port_ok, node->consecutive_failure_count_);
    }
    return;
  }

  if (node->last_restart_time_.nanoseconds() > 0) {
    double elapsed = (node->now() - node->last_restart_time_).seconds();
    if (elapsed < node->restart_cooldown_) {
      PKA_WARN("serial_node",
        "[health] Unhealthy but in cooldown, remaining={:.1f}s",
        node->restart_cooldown_ - elapsed);
      return;
    }
  }

  if (node->restart_in_progress_.load()) {
    PKA_WARN("serial_node", "[health] Restart already in progress, skipping");
    return;
  }

  PKA_WARN("serial_node",
    "[health] Unhealthy: port_open={} is_healthy={} failures={}/{} → restart #{}/{}",
    port_ok, node->is_healthy_,
    node->consecutive_failure_count_, node->max_failure_count_,
    node->total_restart_attempts_ + 1, node->max_restart_attempts_);

  std::thread([node]() {
    if (restart_serial(node)) {
      PKA_INFO("serial_node",
        "[health] Restart succeeded (attempt #{})", node->total_restart_attempts_);
    } else {
      PKA_ERROR("serial_node",
        "[health] Restart failed (attempt #{})", node->total_restart_attempts_);
    }
  }).detach();
}

// ─────────────────────────────────────────────────────────────────────────────
//  串口重启（协议无关）
// ─────────────────────────────────────────────────────────────────────────────

bool UARTNodeTool::restart_serial(UARTNode * node)
{
  if (!node->enable_auto_restart_) { return false; }

  node->restart_in_progress_.store(true);
  node->last_restart_time_ = node->now();
  node->total_restart_attempts_++;

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
    "[restart] Waiting {}ms before reopen (error={}) [non-blocking thread]",
    delay_ms, getErrorString(node->last_error_code_));

  std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  node->init_serial();

  bool ok = node->serial_ && node->serial_->isOpen() && node->is_healthy_;
  if (ok) {
    node->consecutive_failure_count_ = 0;
    // 串口重启后重置 mode 缓存，确保下一帧能重新通知 armor_detector
    node->last_sent_mode_ = 255;
  }

  node->restart_in_progress_.store(false);
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