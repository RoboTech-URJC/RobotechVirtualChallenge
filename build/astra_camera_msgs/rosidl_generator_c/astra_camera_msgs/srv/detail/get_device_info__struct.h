// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astra_camera_msgs:srv/GetDeviceInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "astra_camera_msgs/srv/get_device_info.h"


#ifndef ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_DEVICE_INFO__STRUCT_H_
#define ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_DEVICE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetDeviceInfo in the package astra_camera_msgs.
typedef struct astra_camera_msgs__srv__GetDeviceInfo_Request
{
  uint8_t structure_needs_at_least_one_member;
} astra_camera_msgs__srv__GetDeviceInfo_Request;

// Struct for a sequence of astra_camera_msgs__srv__GetDeviceInfo_Request.
typedef struct astra_camera_msgs__srv__GetDeviceInfo_Request__Sequence
{
  astra_camera_msgs__srv__GetDeviceInfo_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astra_camera_msgs__srv__GetDeviceInfo_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "astra_camera_msgs/msg/detail/device_info__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetDeviceInfo in the package astra_camera_msgs.
typedef struct astra_camera_msgs__srv__GetDeviceInfo_Response
{
  astra_camera_msgs__msg__DeviceInfo info;
  bool success;
  rosidl_runtime_c__String message;
} astra_camera_msgs__srv__GetDeviceInfo_Response;

// Struct for a sequence of astra_camera_msgs__srv__GetDeviceInfo_Response.
typedef struct astra_camera_msgs__srv__GetDeviceInfo_Response__Sequence
{
  astra_camera_msgs__srv__GetDeviceInfo_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astra_camera_msgs__srv__GetDeviceInfo_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  astra_camera_msgs__srv__GetDeviceInfo_Event__request__MAX_SIZE = 1
};
// response
enum
{
  astra_camera_msgs__srv__GetDeviceInfo_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetDeviceInfo in the package astra_camera_msgs.
typedef struct astra_camera_msgs__srv__GetDeviceInfo_Event
{
  service_msgs__msg__ServiceEventInfo info;
  astra_camera_msgs__srv__GetDeviceInfo_Request__Sequence request;
  astra_camera_msgs__srv__GetDeviceInfo_Response__Sequence response;
} astra_camera_msgs__srv__GetDeviceInfo_Event;

// Struct for a sequence of astra_camera_msgs__srv__GetDeviceInfo_Event.
typedef struct astra_camera_msgs__srv__GetDeviceInfo_Event__Sequence
{
  astra_camera_msgs__srv__GetDeviceInfo_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astra_camera_msgs__srv__GetDeviceInfo_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_DEVICE_INFO__STRUCT_H_
