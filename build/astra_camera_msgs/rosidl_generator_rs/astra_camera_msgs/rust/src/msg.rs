#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to astra_camera_msgs__msg__DeviceInfo

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DeviceInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vid: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pid: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub serial_number: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub firmware_version: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub supported_min_sdk_version: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub hardware_version: std::string::String,

}



impl Default for DeviceInfo {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DeviceInfo::default())
  }
}

impl rosidl_runtime_rs::Message for DeviceInfo {
  type RmwMsg = super::msg::rmw::DeviceInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        name: msg.name.as_str().into(),
        vid: msg.vid,
        pid: msg.pid,
        serial_number: msg.serial_number.as_str().into(),
        firmware_version: msg.firmware_version.as_str().into(),
        supported_min_sdk_version: msg.supported_min_sdk_version.as_str().into(),
        hardware_version: msg.hardware_version.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        name: msg.name.as_str().into(),
      vid: msg.vid,
      pid: msg.pid,
        serial_number: msg.serial_number.as_str().into(),
        firmware_version: msg.firmware_version.as_str().into(),
        supported_min_sdk_version: msg.supported_min_sdk_version.as_str().into(),
        hardware_version: msg.hardware_version.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      name: msg.name.to_string(),
      vid: msg.vid,
      pid: msg.pid,
      serial_number: msg.serial_number.to_string(),
      firmware_version: msg.firmware_version.to_string(),
      supported_min_sdk_version: msg.supported_min_sdk_version.to_string(),
      hardware_version: msg.hardware_version.to_string(),
    }
  }
}


// Corresponds to astra_camera_msgs__msg__Extrinsics

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Extrinsics {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rotation: [f64; 9],


    // This member is not documented.
    #[allow(missing_docs)]
    pub translation: [f64; 3],

}



impl Default for Extrinsics {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Extrinsics::default())
  }
}

impl rosidl_runtime_rs::Message for Extrinsics {
  type RmwMsg = super::msg::rmw::Extrinsics;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        rotation: msg.rotation,
        translation: msg.translation,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        rotation: msg.rotation,
        translation: msg.translation,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      rotation: msg.rotation,
      translation: msg.translation,
    }
  }
}


// Corresponds to astra_camera_msgs__msg__Metadata

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Metadata {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub json_data: std::string::String,

}



impl Default for Metadata {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Metadata::default())
  }
}

impl rosidl_runtime_rs::Message for Metadata {
  type RmwMsg = super::msg::rmw::Metadata;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        json_data: msg.json_data.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        json_data: msg.json_data.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      json_data: msg.json_data.to_string(),
    }
  }
}


