#ifndef RM_SERIAL_DRIVER_PROTOCOL_PROTOCOL_FACTORY_HPP_
#define RM_SERIAL_DRIVER_PROTOCOL_PROTOCOL_FACTORY_HPP_

#include "rm_serial_driver/protocol/uart_protocol_base.hpp"
#include "rm_serial_driver/protocol/sentry_protocol.hpp"
#include "rm_serial_driver/protocol/infantry_protocol.hpp"

#include <memory>
#include <string>
#include <stdexcept>

namespace pka {

// ── 机器人类型枚举 ────────────────────────────────────────────────────────────
enum class RobotType {
  SENTRY   = 0,   // 哨兵
  INFANTRY = 1,   // 步兵
};

// ── 从字符串解析 RobotType ────────────────────────────────────────────────────
inline RobotType robot_type_from_string(const std::string & s)
{
  if (s == "sentry"   || s == "SENTRY")   { return RobotType::SENTRY;   }
  if (s == "infantry" || s == "INFANTRY") { return RobotType::INFANTRY; }
  throw std::invalid_argument("Unknown robot_type: '" + s + "'. Valid: sentry, infantry");
}

inline const char * robot_type_to_string(RobotType t)
{
  switch (t) {
    case RobotType::SENTRY:   return "sentry";
    case RobotType::INFANTRY: return "infantry";
    default:                  return "unknown";
  }
}

// ── 协议工厂 ──────────────────────────────────────────────────────────────────
//
// 用法：
//   auto proto = ProtocolFactory::create(RobotType::SENTRY);
//   // 或
//   auto proto = ProtocolFactory::create("infantry");
//
// 返回 IUARTProtocol 实例（unique_ptr），调用方通过虚接口获取包长，
// 具体打包/解包调用对应的静态方法（SentryProtocol::pack / InfantryProtocol::pack）。
//
class ProtocolFactory
{
public:
  static std::unique_ptr<IUARTProtocol> create(RobotType type)
  {
    switch (type) {
      case RobotType::SENTRY:
        return std::make_unique<SentryProtocol>();
      case RobotType::INFANTRY:
        return std::make_unique<InfantryProtocol>();
      default:
        throw std::invalid_argument("ProtocolFactory: unsupported RobotType");
    }
  }

  static std::unique_ptr<IUARTProtocol> create(const std::string & type_str)
  {
    return create(robot_type_from_string(type_str));
  }
};

}  // namespace pka

#endif  // RM_SERIAL_DRIVER_PROTOCOL_PROTOCOL_FACTORY_HPP_