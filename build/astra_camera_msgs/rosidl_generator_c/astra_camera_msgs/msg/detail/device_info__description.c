// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from astra_camera_msgs:msg/DeviceInfo.idl
// generated code does not contain a copyright notice

#include "astra_camera_msgs/msg/detail/device_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_astra_camera_msgs
const rosidl_type_hash_t *
astra_camera_msgs__msg__DeviceInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd9, 0x51, 0xde, 0x7a, 0x35, 0x70, 0xa8, 0x7a,
      0xaf, 0x11, 0xd5, 0x26, 0xd7, 0x44, 0x7a, 0x5c,
      0x91, 0xfc, 0xfb, 0xdb, 0x87, 0x5a, 0x66, 0xd3,
      0x86, 0x79, 0x0d, 0x04, 0xdd, 0x85, 0xb0, 0xe6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char astra_camera_msgs__msg__DeviceInfo__TYPE_NAME[] = "astra_camera_msgs/msg/DeviceInfo";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__header[] = "header";
static char astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__name[] = "name";
static char astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__vid[] = "vid";
static char astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__pid[] = "pid";
static char astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__serial_number[] = "serial_number";
static char astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__firmware_version[] = "firmware_version";
static char astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__supported_min_sdk_version[] = "supported_min_sdk_version";
static char astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__hardware_version[] = "hardware_version";

static rosidl_runtime_c__type_description__Field astra_camera_msgs__msg__DeviceInfo__FIELDS[] = {
  {
    {astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__vid, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__pid, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__serial_number, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__firmware_version, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__supported_min_sdk_version, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {astra_camera_msgs__msg__DeviceInfo__FIELD_NAME__hardware_version, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription astra_camera_msgs__msg__DeviceInfo__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
astra_camera_msgs__msg__DeviceInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {astra_camera_msgs__msg__DeviceInfo__TYPE_NAME, 32, 32},
      {astra_camera_msgs__msg__DeviceInfo__FIELDS, 8, 8},
    },
    {astra_camera_msgs__msg__DeviceInfo__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "string name\n"
  "int32 vid\n"
  "int32 pid\n"
  "string serial_number\n"
  "string firmware_version\n"
  "string supported_min_sdk_version\n"
  "string hardware_version";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
astra_camera_msgs__msg__DeviceInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {astra_camera_msgs__msg__DeviceInfo__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 157, 157},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
astra_camera_msgs__msg__DeviceInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *astra_camera_msgs__msg__DeviceInfo__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
