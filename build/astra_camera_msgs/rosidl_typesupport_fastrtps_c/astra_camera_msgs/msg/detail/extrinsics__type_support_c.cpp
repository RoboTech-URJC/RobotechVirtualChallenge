// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from astra_camera_msgs:msg/Extrinsics.idl
// generated code does not contain a copyright notice
#include "astra_camera_msgs/msg/detail/extrinsics__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "astra_camera_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "astra_camera_msgs/msg/detail/extrinsics__struct.h"
#include "astra_camera_msgs/msg/detail/extrinsics__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_astra_camera_msgs
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_astra_camera_msgs
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_astra_camera_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_astra_camera_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_astra_camera_msgs
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_astra_camera_msgs
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_astra_camera_msgs
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_astra_camera_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _Extrinsics__ros_msg_type = astra_camera_msgs__msg__Extrinsics;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
bool cdr_serialize_astra_camera_msgs__msg__Extrinsics(
  const astra_camera_msgs__msg__Extrinsics * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: rotation
  {
    size_t size = 9;
    auto array_ptr = ros_message->rotation;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: translation
  {
    size_t size = 3;
    auto array_ptr = ros_message->translation;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
bool cdr_deserialize_astra_camera_msgs__msg__Extrinsics(
  eprosima::fastcdr::Cdr & cdr,
  astra_camera_msgs__msg__Extrinsics * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: rotation
  {
    size_t size = 9;
    auto array_ptr = ros_message->rotation;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: translation
  {
    size_t size = 3;
    auto array_ptr = ros_message->translation;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
size_t get_serialized_size_astra_camera_msgs__msg__Extrinsics(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Extrinsics__ros_msg_type * ros_message = static_cast<const _Extrinsics__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: rotation
  {
    size_t array_size = 9;
    auto array_ptr = ros_message->rotation;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: translation
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->translation;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
size_t max_serialized_size_astra_camera_msgs__msg__Extrinsics(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: rotation
  {
    size_t array_size = 9;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: translation
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = astra_camera_msgs__msg__Extrinsics;
    is_plain =
      (
      offsetof(DataType, translation) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
bool cdr_serialize_key_astra_camera_msgs__msg__Extrinsics(
  const astra_camera_msgs__msg__Extrinsics * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: rotation
  {
    size_t size = 9;
    auto array_ptr = ros_message->rotation;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: translation
  {
    size_t size = 3;
    auto array_ptr = ros_message->translation;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
size_t get_serialized_size_key_astra_camera_msgs__msg__Extrinsics(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Extrinsics__ros_msg_type * ros_message = static_cast<const _Extrinsics__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: rotation
  {
    size_t array_size = 9;
    auto array_ptr = ros_message->rotation;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: translation
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->translation;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_astra_camera_msgs
size_t max_serialized_size_key_astra_camera_msgs__msg__Extrinsics(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: rotation
  {
    size_t array_size = 9;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: translation
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = astra_camera_msgs__msg__Extrinsics;
    is_plain =
      (
      offsetof(DataType, translation) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Extrinsics__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const astra_camera_msgs__msg__Extrinsics * ros_message = static_cast<const astra_camera_msgs__msg__Extrinsics *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_astra_camera_msgs__msg__Extrinsics(ros_message, cdr);
}

static bool _Extrinsics__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  astra_camera_msgs__msg__Extrinsics * ros_message = static_cast<astra_camera_msgs__msg__Extrinsics *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_astra_camera_msgs__msg__Extrinsics(cdr, ros_message);
}

static uint32_t _Extrinsics__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_astra_camera_msgs__msg__Extrinsics(
      untyped_ros_message, 0));
}

static size_t _Extrinsics__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_astra_camera_msgs__msg__Extrinsics(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Extrinsics = {
  "astra_camera_msgs::msg",
  "Extrinsics",
  _Extrinsics__cdr_serialize,
  _Extrinsics__cdr_deserialize,
  _Extrinsics__get_serialized_size,
  _Extrinsics__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Extrinsics__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Extrinsics,
  get_message_typesupport_handle_function,
  &astra_camera_msgs__msg__Extrinsics__get_type_hash,
  &astra_camera_msgs__msg__Extrinsics__get_type_description,
  &astra_camera_msgs__msg__Extrinsics__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, astra_camera_msgs, msg, Extrinsics)() {
  return &_Extrinsics__type_support;
}

#if defined(__cplusplus)
}
#endif
