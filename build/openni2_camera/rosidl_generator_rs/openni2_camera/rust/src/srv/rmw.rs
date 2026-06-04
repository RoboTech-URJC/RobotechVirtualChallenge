#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "openni2_camera__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__openni2_camera__srv__GetSerial_Request() -> *const std::ffi::c_void;
}

#[link(name = "openni2_camera__rosidl_generator_c")]
extern "C" {
    fn openni2_camera__srv__GetSerial_Request__init(msg: *mut GetSerial_Request) -> bool;
    fn openni2_camera__srv__GetSerial_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetSerial_Request>, size: usize) -> bool;
    fn openni2_camera__srv__GetSerial_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetSerial_Request>);
    fn openni2_camera__srv__GetSerial_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetSerial_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetSerial_Request>) -> bool;
}

// Corresponds to openni2_camera__srv__GetSerial_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSerial_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetSerial_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !openni2_camera__srv__GetSerial_Request__init(&mut msg as *mut _) {
        panic!("Call to openni2_camera__srv__GetSerial_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetSerial_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { openni2_camera__srv__GetSerial_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { openni2_camera__srv__GetSerial_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { openni2_camera__srv__GetSerial_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetSerial_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetSerial_Request where Self: Sized {
  const TYPE_NAME: &'static str = "openni2_camera/srv/GetSerial_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__openni2_camera__srv__GetSerial_Request() }
  }
}


#[link(name = "openni2_camera__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__openni2_camera__srv__GetSerial_Response() -> *const std::ffi::c_void;
}

#[link(name = "openni2_camera__rosidl_generator_c")]
extern "C" {
    fn openni2_camera__srv__GetSerial_Response__init(msg: *mut GetSerial_Response) -> bool;
    fn openni2_camera__srv__GetSerial_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetSerial_Response>, size: usize) -> bool;
    fn openni2_camera__srv__GetSerial_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetSerial_Response>);
    fn openni2_camera__srv__GetSerial_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetSerial_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetSerial_Response>) -> bool;
}

// Corresponds to openni2_camera__srv__GetSerial_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSerial_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub serial: rosidl_runtime_rs::String,

}



impl Default for GetSerial_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !openni2_camera__srv__GetSerial_Response__init(&mut msg as *mut _) {
        panic!("Call to openni2_camera__srv__GetSerial_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetSerial_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { openni2_camera__srv__GetSerial_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { openni2_camera__srv__GetSerial_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { openni2_camera__srv__GetSerial_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetSerial_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetSerial_Response where Self: Sized {
  const TYPE_NAME: &'static str = "openni2_camera/srv/GetSerial_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__openni2_camera__srv__GetSerial_Response() }
  }
}






#[link(name = "openni2_camera__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__openni2_camera__srv__GetSerial() -> *const std::ffi::c_void;
}

// Corresponds to openni2_camera__srv__GetSerial
#[allow(missing_docs, non_camel_case_types)]
pub struct GetSerial;

impl rosidl_runtime_rs::Service for GetSerial {
    type Request = GetSerial_Request;
    type Response = GetSerial_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__openni2_camera__srv__GetSerial() }
    }
}


