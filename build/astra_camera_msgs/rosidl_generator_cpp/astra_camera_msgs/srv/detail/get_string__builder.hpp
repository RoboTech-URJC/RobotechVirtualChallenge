// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astra_camera_msgs:srv/GetString.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "astra_camera_msgs/srv/get_string.hpp"


#ifndef ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_STRING__BUILDER_HPP_
#define ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_STRING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astra_camera_msgs/srv/detail/get_string__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astra_camera_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astra_camera_msgs::srv::GetString_Request>()
{
  return ::astra_camera_msgs::srv::GetString_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace astra_camera_msgs


namespace astra_camera_msgs
{

namespace srv
{

namespace builder
{

class Init_GetString_Response_message
{
public:
  explicit Init_GetString_Response_message(::astra_camera_msgs::srv::GetString_Response & msg)
  : msg_(msg)
  {}
  ::astra_camera_msgs::srv::GetString_Response message(::astra_camera_msgs::srv::GetString_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astra_camera_msgs::srv::GetString_Response msg_;
};

class Init_GetString_Response_success
{
public:
  explicit Init_GetString_Response_success(::astra_camera_msgs::srv::GetString_Response & msg)
  : msg_(msg)
  {}
  Init_GetString_Response_message success(::astra_camera_msgs::srv::GetString_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetString_Response_message(msg_);
  }

private:
  ::astra_camera_msgs::srv::GetString_Response msg_;
};

class Init_GetString_Response_data
{
public:
  Init_GetString_Response_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetString_Response_success data(::astra_camera_msgs::srv::GetString_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_GetString_Response_success(msg_);
  }

private:
  ::astra_camera_msgs::srv::GetString_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astra_camera_msgs::srv::GetString_Response>()
{
  return astra_camera_msgs::srv::builder::Init_GetString_Response_data();
}

}  // namespace astra_camera_msgs


namespace astra_camera_msgs
{

namespace srv
{

namespace builder
{

class Init_GetString_Event_response
{
public:
  explicit Init_GetString_Event_response(::astra_camera_msgs::srv::GetString_Event & msg)
  : msg_(msg)
  {}
  ::astra_camera_msgs::srv::GetString_Event response(::astra_camera_msgs::srv::GetString_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astra_camera_msgs::srv::GetString_Event msg_;
};

class Init_GetString_Event_request
{
public:
  explicit Init_GetString_Event_request(::astra_camera_msgs::srv::GetString_Event & msg)
  : msg_(msg)
  {}
  Init_GetString_Event_response request(::astra_camera_msgs::srv::GetString_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetString_Event_response(msg_);
  }

private:
  ::astra_camera_msgs::srv::GetString_Event msg_;
};

class Init_GetString_Event_info
{
public:
  Init_GetString_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetString_Event_request info(::astra_camera_msgs::srv::GetString_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetString_Event_request(msg_);
  }

private:
  ::astra_camera_msgs::srv::GetString_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::astra_camera_msgs::srv::GetString_Event>()
{
  return astra_camera_msgs::srv::builder::Init_GetString_Event_info();
}

}  // namespace astra_camera_msgs

#endif  // ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_STRING__BUILDER_HPP_
