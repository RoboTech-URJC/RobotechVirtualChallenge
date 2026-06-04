#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "astra_camera_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__astra_camera_msgs__msg__DeviceInfo() -> *const std::ffi::c_void;
}

#[link(name = "astra_camera_msgs__rosidl_generator_c")]
extern "C" {
    fn astra_camera_msgs__msg__DeviceInfo__init(msg: *mut DeviceInfo) -> bool;
    fn astra_camera_msgs__msg__DeviceInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DeviceInfo>, size: usize) -> bool;
    fn astra_camera_msgs__msg__DeviceInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DeviceInfo>);
    fn astra_camera_msgs__msg__DeviceInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DeviceInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<DeviceInfo>) -> bool;
}

// Corresponds to astra_camera_msgs__msg__DeviceInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DeviceInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vid: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pid: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub serial_number: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub firmware_version: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub supported_min_sdk_version: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub hardware_version: rosidl_runtime_rs::String,

}



impl Default for DeviceInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !astra_camera_msgs__msg__DeviceInfo__init(&mut msg as *mut _) {
        panic!("Call to astra_camera_msgs__msg__DeviceInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DeviceInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { astra_camera_msgs__msg__DeviceInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { astra_camera_msgs__msg__DeviceInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { astra_camera_msgs__msg__DeviceInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DeviceInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DeviceInfo where Self: Sized {
  const TYPE_NAME: &'static str = "astra_camera_msgs/msg/DeviceInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__astra_camera_msgs__msg__DeviceInfo() }
  }
}


#[link(name = "astra_camera_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__astra_camera_msgs__msg__Extrinsics() -> *const std::ffi::c_void;
}

#[link(name = "astra_camera_msgs__rosidl_generator_c")]
extern "C" {
    fn astra_camera_msgs__msg__Extrinsics__init(msg: *mut Extrinsics) -> bool;
    fn astra_camera_msgs__msg__Extrinsics__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Extrinsics>, size: usize) -> bool;
    fn astra_camera_msgs__msg__Extrinsics__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Extrinsics>);
    fn astra_camera_msgs__msg__Extrinsics__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Extrinsics>, out_seq: *mut rosidl_runtime_rs::Sequence<Extrinsics>) -> bool;
}

// Corresponds to astra_camera_msgs__msg__Extrinsics
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Extrinsics {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rotation: [f64; 9],


    // This member is not documented.
    #[allow(missing_docs)]
    pub translation: [f64; 3],

}



impl Default for Extrinsics {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !astra_camera_msgs__msg__Extrinsics__init(&mut msg as *mut _) {
        panic!("Call to astra_camera_msgs__msg__Extrinsics__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Extrinsics {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { astra_camera_msgs__msg__Extrinsics__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { astra_camera_msgs__msg__Extrinsics__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { astra_camera_msgs__msg__Extrinsics__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Extrinsics {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Extrinsics where Self: Sized {
  const TYPE_NAME: &'static str = "astra_camera_msgs/msg/Extrinsics";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__astra_camera_msgs__msg__Extrinsics() }
  }
}


#[link(name = "astra_camera_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__astra_camera_msgs__msg__Metadata() -> *const std::ffi::c_void;
}

#[link(name = "astra_camera_msgs__rosidl_generator_c")]
extern "C" {
    fn astra_camera_msgs__msg__Metadata__init(msg: *mut Metadata) -> bool;
    fn astra_camera_msgs__msg__Metadata__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Metadata>, size: usize) -> bool;
    fn astra_camera_msgs__msg__Metadata__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Metadata>);
    fn astra_camera_msgs__msg__Metadata__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Metadata>, out_seq: *mut rosidl_runtime_rs::Sequence<Metadata>) -> bool;
}

// Corresponds to astra_camera_msgs__msg__Metadata
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Metadata {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub json_data: rosidl_runtime_rs::String,

}



impl Default for Metadata {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !astra_camera_msgs__msg__Metadata__init(&mut msg as *mut _) {
        panic!("Call to astra_camera_msgs__msg__Metadata__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Metadata {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { astra_camera_msgs__msg__Metadata__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { astra_camera_msgs__msg__Metadata__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { astra_camera_msgs__msg__Metadata__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Metadata {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Metadata where Self: Sized {
  const TYPE_NAME: &'static str = "astra_camera_msgs/msg/Metadata";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__astra_camera_msgs__msg__Metadata() }
  }
}


