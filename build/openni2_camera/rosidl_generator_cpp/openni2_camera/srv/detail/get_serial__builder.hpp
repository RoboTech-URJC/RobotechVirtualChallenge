// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from openni2_camera:srv/GetSerial.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "openni2_camera/srv/get_serial.hpp"


#ifndef OPENNI2_CAMERA__SRV__DETAIL__GET_SERIAL__BUILDER_HPP_
#define OPENNI2_CAMERA__SRV__DETAIL__GET_SERIAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "openni2_camera/srv/detail/get_serial__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace openni2_camera
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::openni2_camera::srv::GetSerial_Request>()
{
  return ::openni2_camera::srv::GetSerial_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace openni2_camera


namespace openni2_camera
{

namespace srv
{

namespace builder
{

class Init_GetSerial_Response_serial
{
public:
  Init_GetSerial_Response_serial()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::openni2_camera::srv::GetSerial_Response serial(::openni2_camera::srv::GetSerial_Response::_serial_type arg)
  {
    msg_.serial = std::move(arg);
    return std::move(msg_);
  }

private:
  ::openni2_camera::srv::GetSerial_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::openni2_camera::srv::GetSerial_Response>()
{
  return openni2_camera::srv::builder::Init_GetSerial_Response_serial();
}

}  // namespace openni2_camera


namespace openni2_camera
{

namespace srv
{

namespace builder
{

class Init_GetSerial_Event_response
{
public:
  explicit Init_GetSerial_Event_response(::openni2_camera::srv::GetSerial_Event & msg)
  : msg_(msg)
  {}
  ::openni2_camera::srv::GetSerial_Event response(::openni2_camera::srv::GetSerial_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::openni2_camera::srv::GetSerial_Event msg_;
};

class Init_GetSerial_Event_request
{
public:
  explicit Init_GetSerial_Event_request(::openni2_camera::srv::GetSerial_Event & msg)
  : msg_(msg)
  {}
  Init_GetSerial_Event_response request(::openni2_camera::srv::GetSerial_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetSerial_Event_response(msg_);
  }

private:
  ::openni2_camera::srv::GetSerial_Event msg_;
};

class Init_GetSerial_Event_info
{
public:
  Init_GetSerial_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetSerial_Event_request info(::openni2_camera::srv::GetSerial_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetSerial_Event_request(msg_);
  }

private:
  ::openni2_camera::srv::GetSerial_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::openni2_camera::srv::GetSerial_Event>()
{
  return openni2_camera::srv::builder::Init_GetSerial_Event_info();
}

}  // namespace openni2_camera

#endif  // OPENNI2_CAMERA__SRV__DETAIL__GET_SERIAL__BUILDER_HPP_
