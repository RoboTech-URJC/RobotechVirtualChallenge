// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from openni2_camera:srv/GetSerial.idl
// generated code does not contain a copyright notice

#include "openni2_camera/srv/detail/get_serial__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_openni2_camera
const rosidl_type_hash_t *
openni2_camera__srv__GetSerial__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb9, 0x57, 0x9c, 0x67, 0x10, 0x2f, 0xf0, 0x3f,
      0xb8, 0x2e, 0xf7, 0x27, 0x72, 0x80, 0x51, 0x16,
      0xe7, 0x22, 0x5d, 0x40, 0x4c, 0x88, 0xad, 0xde,
      0x73, 0x91, 0x43, 0xdb, 0x34, 0x6b, 0x93, 0x8f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_openni2_camera
const rosidl_type_hash_t *
openni2_camera__srv__GetSerial_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x50, 0xa4, 0xa1, 0xf0, 0xa4, 0xe0, 0xfc, 0x69,
      0x2b, 0xc9, 0xce, 0xaf, 0x69, 0x06, 0x41, 0x06,
      0x0f, 0x7d, 0xfe, 0x45, 0x31, 0x11, 0x0a, 0xff,
      0x46, 0x63, 0x80, 0x90, 0x65, 0xcf, 0x9c, 0x31,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_openni2_camera
const rosidl_type_hash_t *
openni2_camera__srv__GetSerial_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x80, 0x6e, 0xb9, 0xc6, 0xf9, 0x9b, 0xef, 0x8a,
      0x7a, 0xd2, 0x2b, 0x9d, 0x4d, 0x04, 0x07, 0xe3,
      0xfe, 0x48, 0x3e, 0xad, 0x21, 0xa0, 0x84, 0x95,
      0xa3, 0xe8, 0xbe, 0xea, 0x22, 0xee, 0x9c, 0xed,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_openni2_camera
const rosidl_type_hash_t *
openni2_camera__srv__GetSerial_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd1, 0x8e, 0x35, 0xce, 0x36, 0x56, 0x13, 0x78,
      0x51, 0xa9, 0x07, 0x1c, 0x2c, 0xd5, 0xf4, 0xde,
      0x14, 0x13, 0x72, 0x0e, 0x61, 0x8c, 0x5a, 0xbc,
      0x90, 0x20, 0x54, 0x82, 0x43, 0xc5, 0x81, 0xed,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char openni2_camera__srv__GetSerial__TYPE_NAME[] = "openni2_camera/srv/GetSerial";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char openni2_camera__srv__GetSerial_Event__TYPE_NAME[] = "openni2_camera/srv/GetSerial_Event";
static char openni2_camera__srv__GetSerial_Request__TYPE_NAME[] = "openni2_camera/srv/GetSerial_Request";
static char openni2_camera__srv__GetSerial_Response__TYPE_NAME[] = "openni2_camera/srv/GetSerial_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char openni2_camera__srv__GetSerial__FIELD_NAME__request_message[] = "request_message";
static char openni2_camera__srv__GetSerial__FIELD_NAME__response_message[] = "response_message";
static char openni2_camera__srv__GetSerial__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field openni2_camera__srv__GetSerial__FIELDS[] = {
  {
    {openni2_camera__srv__GetSerial__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {openni2_camera__srv__GetSerial_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {openni2_camera__srv__GetSerial__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {openni2_camera__srv__GetSerial_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {openni2_camera__srv__GetSerial__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {openni2_camera__srv__GetSerial_Event__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription openni2_camera__srv__GetSerial__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {openni2_camera__srv__GetSerial_Event__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {openni2_camera__srv__GetSerial_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {openni2_camera__srv__GetSerial_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
openni2_camera__srv__GetSerial__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {openni2_camera__srv__GetSerial__TYPE_NAME, 28, 28},
      {openni2_camera__srv__GetSerial__FIELDS, 3, 3},
    },
    {openni2_camera__srv__GetSerial__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = openni2_camera__srv__GetSerial_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = openni2_camera__srv__GetSerial_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = openni2_camera__srv__GetSerial_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char openni2_camera__srv__GetSerial_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field openni2_camera__srv__GetSerial_Request__FIELDS[] = {
  {
    {openni2_camera__srv__GetSerial_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
openni2_camera__srv__GetSerial_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {openni2_camera__srv__GetSerial_Request__TYPE_NAME, 36, 36},
      {openni2_camera__srv__GetSerial_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char openni2_camera__srv__GetSerial_Response__FIELD_NAME__serial[] = "serial";

static rosidl_runtime_c__type_description__Field openni2_camera__srv__GetSerial_Response__FIELDS[] = {
  {
    {openni2_camera__srv__GetSerial_Response__FIELD_NAME__serial, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
openni2_camera__srv__GetSerial_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {openni2_camera__srv__GetSerial_Response__TYPE_NAME, 37, 37},
      {openni2_camera__srv__GetSerial_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char openni2_camera__srv__GetSerial_Event__FIELD_NAME__info[] = "info";
static char openni2_camera__srv__GetSerial_Event__FIELD_NAME__request[] = "request";
static char openni2_camera__srv__GetSerial_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field openni2_camera__srv__GetSerial_Event__FIELDS[] = {
  {
    {openni2_camera__srv__GetSerial_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {openni2_camera__srv__GetSerial_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {openni2_camera__srv__GetSerial_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {openni2_camera__srv__GetSerial_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {openni2_camera__srv__GetSerial_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription openni2_camera__srv__GetSerial_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {openni2_camera__srv__GetSerial_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {openni2_camera__srv__GetSerial_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
openni2_camera__srv__GetSerial_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {openni2_camera__srv__GetSerial_Event__TYPE_NAME, 34, 34},
      {openni2_camera__srv__GetSerial_Event__FIELDS, 3, 3},
    },
    {openni2_camera__srv__GetSerial_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = openni2_camera__srv__GetSerial_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = openni2_camera__srv__GetSerial_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "string serial";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
openni2_camera__srv__GetSerial__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {openni2_camera__srv__GetSerial__TYPE_NAME, 28, 28},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 18, 18},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
openni2_camera__srv__GetSerial_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {openni2_camera__srv__GetSerial_Request__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
openni2_camera__srv__GetSerial_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {openni2_camera__srv__GetSerial_Response__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
openni2_camera__srv__GetSerial_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {openni2_camera__srv__GetSerial_Event__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
openni2_camera__srv__GetSerial__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *openni2_camera__srv__GetSerial__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *openni2_camera__srv__GetSerial_Event__get_individual_type_description_source(NULL);
    sources[3] = *openni2_camera__srv__GetSerial_Request__get_individual_type_description_source(NULL);
    sources[4] = *openni2_camera__srv__GetSerial_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
openni2_camera__srv__GetSerial_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *openni2_camera__srv__GetSerial_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
openni2_camera__srv__GetSerial_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *openni2_camera__srv__GetSerial_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
openni2_camera__srv__GetSerial_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *openni2_camera__srv__GetSerial_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *openni2_camera__srv__GetSerial_Request__get_individual_type_description_source(NULL);
    sources[3] = *openni2_camera__srv__GetSerial_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
