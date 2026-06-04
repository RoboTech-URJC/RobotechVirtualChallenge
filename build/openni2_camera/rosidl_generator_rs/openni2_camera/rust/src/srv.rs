#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to openni2_camera__srv__GetSerial_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSerial_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetSerial_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetSerial_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetSerial_Request {
  type RmwMsg = super::srv::rmw::GetSerial_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to openni2_camera__srv__GetSerial_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSerial_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub serial: std::string::String,

}



impl Default for GetSerial_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetSerial_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetSerial_Response {
  type RmwMsg = super::srv::rmw::GetSerial_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        serial: msg.serial.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        serial: msg.serial.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      serial: msg.serial.to_string(),
    }
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


