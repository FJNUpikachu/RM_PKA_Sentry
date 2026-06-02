#ifndef RM_SERIAL_DRIVER_ERROR_CODES_HPP_
#define RM_SERIAL_DRIVER_ERROR_CODES_HPP_

namespace pka {

enum class SerialErrorCode {
  OK = 0,
  DEVICE_NOT_FOUND,
  PERMISSION_DENIED,
  DEVICE_BUSY,
  TIMEOUT,
  INVALID_PARAMETER,
  FRAME_ERROR,
  HARDWARE_ERROR,
  UNKNOWN_ERROR
};

inline const char * getErrorString(SerialErrorCode code)
{
  switch (code) {
    case SerialErrorCode::OK:                return "OK";
    case SerialErrorCode::DEVICE_NOT_FOUND:  return "DeviceNotFound";
    case SerialErrorCode::PERMISSION_DENIED: return "PermissionDenied";
    case SerialErrorCode::DEVICE_BUSY:       return "DeviceBusy";
    case SerialErrorCode::TIMEOUT:           return "Timeout";
    case SerialErrorCode::INVALID_PARAMETER: return "InvalidParameter";
    case SerialErrorCode::FRAME_ERROR:       return "FrameError";
    case SerialErrorCode::HARDWARE_ERROR:    return "HardwareError";
    case SerialErrorCode::UNKNOWN_ERROR:     return "UnknownError";
    default:                                 return "Invalid";
  }
}

}  // namespace pka

#endif  // RM_SERIAL_DRIVER_ERROR_CODES_HPP_