// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from astra_camera_msgs:msg/DeviceInfo.idl
// generated code does not contain a copyright notice
#ifndef ASTRA_CAMERA_MSGS__MSG__DETAIL__DEVICE_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ASTRA_CAMERA_MSGS__MSG__DETAIL__DEVICE_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "astra_camera_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "astra_camera_msgs/msg/detail/device_info__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
bool cdr_serialize_astra_camera_msgs__msg__DeviceInfo(
  const astra_camera_msgs__msg__DeviceInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
bool cdr_deserialize_astra_camera_msgs__msg__DeviceInfo(
  eprosima::fastcdr::Cdr &,
  astra_camera_msgs__msg__DeviceInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
size_t get_serialized_size_astra_camera_msgs__msg__DeviceInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
size_t max_serialized_size_astra_camera_msgs__msg__DeviceInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
bool cdr_serialize_key_astra_camera_msgs__msg__DeviceInfo(
  const astra_camera_msgs__msg__DeviceInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
size_t get_serialized_size_key_astra_camera_msgs__msg__DeviceInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
size_t max_serialized_size_key_astra_camera_msgs__msg__DeviceInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, astra_camera_msgs, msg, DeviceInfo)();

#ifdef __cplusplus
}
#endif

#endif  // ASTRA_CAMERA_MSGS__MSG__DETAIL__DEVICE_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
