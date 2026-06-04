#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to astra_camera_msgs__srv__GetDeviceInfo_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetDeviceInfo_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetDeviceInfo_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetDeviceInfo_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetDeviceInfo_Request {
  type RmwMsg = super::srv::rmw::GetDeviceInfo_Request;

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


// Corresponds to astra_camera_msgs__srv__GetDeviceInfo_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetDeviceInfo_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub info: super::msg::DeviceInfo,


    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for GetDeviceInfo_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetDeviceInfo_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetDeviceInfo_Response {
  type RmwMsg = super::srv::rmw::GetDeviceInfo_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        info: super::msg::DeviceInfo::into_rmw_message(std::borrow::Cow::Owned(msg.info)).into_owned(),
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        info: super::msg::DeviceInfo::into_rmw_message(std::borrow::Cow::Borrowed(&msg.info)).into_owned(),
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      info: super::msg::DeviceInfo::from_rmw_message(msg.info),
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to astra_camera_msgs__srv__GetCameraInfo_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCameraInfo_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetCameraInfo_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetCameraInfo_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetCameraInfo_Request {
  type RmwMsg = super::srv::rmw::GetCameraInfo_Request;

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


// Corresponds to astra_camera_msgs__srv__GetCameraInfo_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCameraInfo_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub info: sensor_msgs::msg::CameraInfo,


    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for GetCameraInfo_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetCameraInfo_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetCameraInfo_Response {
  type RmwMsg = super::srv::rmw::GetCameraInfo_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        info: sensor_msgs::msg::CameraInfo::into_rmw_message(std::borrow::Cow::Owned(msg.info)).into_owned(),
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        info: sensor_msgs::msg::CameraInfo::into_rmw_message(std::borrow::Cow::Borrowed(&msg.info)).into_owned(),
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      info: sensor_msgs::msg::CameraInfo::from_rmw_message(msg.info),
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to astra_camera_msgs__srv__GetCameraParams_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCameraParams_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetCameraParams_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetCameraParams_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetCameraParams_Request {
  type RmwMsg = super::srv::rmw::GetCameraParams_Request;

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


// Corresponds to astra_camera_msgs__srv__GetCameraParams_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCameraParams_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub l_intr_p: [f32; 4],


    // This member is not documented.
    #[allow(missing_docs)]
    pub r_intr_p: [f32; 4],


    // This member is not documented.
    #[allow(missing_docs)]
    pub r2l_r: [f32; 9],


    // This member is not documented.
    #[allow(missing_docs)]
    pub r2l_t: [f32; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub l_k: [f32; 5],


    // This member is not documented.
    #[allow(missing_docs)]
    pub r_k: [f32; 5],


    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for GetCameraParams_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetCameraParams_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetCameraParams_Response {
  type RmwMsg = super::srv::rmw::GetCameraParams_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        l_intr_p: msg.l_intr_p,
        r_intr_p: msg.r_intr_p,
        r2l_r: msg.r2l_r,
        r2l_t: msg.r2l_t,
        l_k: msg.l_k,
        r_k: msg.r_k,
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        l_intr_p: msg.l_intr_p,
        r_intr_p: msg.r_intr_p,
        r2l_r: msg.r2l_r,
        r2l_t: msg.r2l_t,
        l_k: msg.l_k,
        r_k: msg.r_k,
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      l_intr_p: msg.l_intr_p,
      r_intr_p: msg.r_intr_p,
      r2l_r: msg.r2l_r,
      r2l_t: msg.r2l_t,
      l_k: msg.l_k,
      r_k: msg.r_k,
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to astra_camera_msgs__srv__GetInt32_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetInt32_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetInt32_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetInt32_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetInt32_Request {
  type RmwMsg = super::srv::rmw::GetInt32_Request;

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


// Corresponds to astra_camera_msgs__srv__GetInt32_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetInt32_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub data: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for GetInt32_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetInt32_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetInt32_Response {
  type RmwMsg = super::srv::rmw::GetInt32_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data,
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      data: msg.data,
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      data: msg.data,
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to astra_camera_msgs__srv__GetString_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetString_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetString_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetString_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetString_Request {
  type RmwMsg = super::srv::rmw::GetString_Request;

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


// Corresponds to astra_camera_msgs__srv__GetString_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetString_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub data: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for GetString_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetString_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetString_Response {
  type RmwMsg = super::srv::rmw::GetString_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data.as_str().into(),
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data.as_str().into(),
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      data: msg.data.to_string(),
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to astra_camera_msgs__srv__SetInt32_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetInt32_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub data: i32,

}



impl Default for SetInt32_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetInt32_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetInt32_Request {
  type RmwMsg = super::srv::rmw::SetInt32_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      data: msg.data,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      data: msg.data,
    }
  }
}


// Corresponds to astra_camera_msgs__srv__SetInt32_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetInt32_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for SetInt32_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetInt32_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetInt32_Response {
  type RmwMsg = super::srv::rmw::SetInt32_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}






#[link(name = "astra_camera_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__GetDeviceInfo() -> *const std::ffi::c_void;
}

// Corresponds to astra_camera_msgs__srv__GetDeviceInfo
#[allow(missing_docs, non_camel_case_types)]
pub struct GetDeviceInfo;

impl rosidl_runtime_rs::Service for GetDeviceInfo {
    type Request = GetDeviceInfo_Request;
    type Response = GetDeviceInfo_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__GetDeviceInfo() }
    }
}




#[link(name = "astra_camera_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__GetCameraInfo() -> *const std::ffi::c_void;
}

// Corresponds to astra_camera_msgs__srv__GetCameraInfo
#[allow(missing_docs, non_camel_case_types)]
pub struct GetCameraInfo;

impl rosidl_runtime_rs::Service for GetCameraInfo {
    type Request = GetCameraInfo_Request;
    type Response = GetCameraInfo_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__GetCameraInfo() }
    }
}




#[link(name = "astra_camera_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__GetCameraParams() -> *const std::ffi::c_void;
}

// Corresponds to astra_camera_msgs__srv__GetCameraParams
#[allow(missing_docs, non_camel_case_types)]
pub struct GetCameraParams;

impl rosidl_runtime_rs::Service for GetCameraParams {
    type Request = GetCameraParams_Request;
    type Response = GetCameraParams_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__GetCameraParams() }
    }
}




#[link(name = "astra_camera_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__GetInt32() -> *const std::ffi::c_void;
}

// Corresponds to astra_camera_msgs__srv__GetInt32
#[allow(missing_docs, non_camel_case_types)]
pub struct GetInt32;

impl rosidl_runtime_rs::Service for GetInt32 {
    type Request = GetInt32_Request;
    type Response = GetInt32_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__GetInt32() }
    }
}




#[link(name = "astra_camera_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__GetString() -> *const std::ffi::c_void;
}

// Corresponds to astra_camera_msgs__srv__GetString
#[allow(missing_docs, non_camel_case_types)]
pub struct GetString;

impl rosidl_runtime_rs::Service for GetString {
    type Request = GetString_Request;
    type Response = GetString_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__GetString() }
    }
}




#[link(name = "astra_camera_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__SetInt32() -> *const std::ffi::c_void;
}

// Corresponds to astra_camera_msgs__srv__SetInt32
#[allow(missing_docs, non_camel_case_types)]
pub struct SetInt32;

impl rosidl_runtime_rs::Service for SetInt32 {
    type Request = SetInt32_Request;
    type Response = SetInt32_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__astra_camera_msgs__srv__SetInt32() }
    }
}


