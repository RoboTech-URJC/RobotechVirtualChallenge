// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from openni2_camera:srv/GetSerial.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "openni2_camera/srv/get_serial.h"


#ifndef OPENNI2_CAMERA__SRV__DETAIL__GET_SERIAL__STRUCT_H_
#define OPENNI2_CAMERA__SRV__DETAIL__GET_SERIAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetSerial in the package openni2_camera.
typedef struct openni2_camera__srv__GetSerial_Request
{
  uint8_t structure_needs_at_least_one_member;
} openni2_camera__srv__GetSerial_Request;

// Struct for a sequence of openni2_camera__srv__GetSerial_Request.
typedef struct openni2_camera__srv__GetSerial_Request__Sequence
{
  openni2_camera__srv__GetSerial_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} openni2_camera__srv__GetSerial_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'serial'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetSerial in the package openni2_camera.
typedef struct openni2_camera__srv__GetSerial_Response
{
  rosidl_runtime_c__String serial;
} openni2_camera__srv__GetSerial_Response;

// Struct for a sequence of openni2_camera__srv__GetSerial_Response.
typedef struct openni2_camera__srv__GetSerial_Response__Sequence
{
  openni2_camera__srv__GetSerial_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} openni2_camera__srv__GetSerial_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  openni2_camera__srv__GetSerial_Event__request__MAX_SIZE = 1
};
// response
enum
{
  openni2_camera__srv__GetSerial_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetSerial in the package openni2_camera.
typedef struct openni2_camera__srv__GetSerial_Event
{
  service_msgs__msg__ServiceEventInfo info;
  openni2_camera__srv__GetSerial_Request__Sequence request;
  openni2_camera__srv__GetSerial_Response__Sequence response;
} openni2_camera__srv__GetSerial_Event;

// Struct for a sequence of openni2_camera__srv__GetSerial_Event.
typedef struct openni2_camera__srv__GetSerial_Event__Sequence
{
  openni2_camera__srv__GetSerial_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} openni2_camera__srv__GetSerial_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OPENNI2_CAMERA__SRV__DETAIL__GET_SERIAL__STRUCT_H_
