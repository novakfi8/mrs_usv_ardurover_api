/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_hw_api/api.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <std_msgs/Float64.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>

//}

namespace mrs_uav_pixhawk_api
{

/* class MrsUavPixhawkApi //{ */

class MrsUavPixhawkApi : public mrs_uav_hw_api::MrsUavHwApi {

public:
  ~MrsUavPixhawkApi(){};

  void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers, const std::string &topic_prefix,
                  const std::string &uav_name);

  // | --------------------- status methods --------------------- |

  mrs_msgs::HwApiDiagnostics getDiagnostics();
  mrs_msgs::HwApiMode        getMode();

  // | --------------------- topic callbacks -------------------- |

  bool callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd &msg);
  bool callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd &msg);
  bool callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd &msg);
  bool callbackTranslationCmd(const mrs_msgs::HwApiTranslationCmd &msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);
  std::tuple<bool, std::string> callbackOffboard(const bool &request);

private:
  bool is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- parameters ----------------------- |

  std::string _topic_mavros_command_long_;
  std::string _topic_mavros_state_;
  std::string _topic_mavros_odometry_local_;
  std::string _topic_mavros_gps_;
  std::string _topic_mavros_distance_sensor_;
  std::string _topic_mavros_imu_;
  std::string _topic_mavros_magnetometer_heading_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mavros_msgs::CommandLong> sch_mavros_command_long_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mavros_msgs::State> sh_mavros_state_;

  void timeoutMavrosState(const std::string &topic, const ros::Time &last_msg, const int n_pubs);
  void callbackMavrosState(mrs_lib::SubscribeHandler<mavros_msgs::State> &wrp);

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mavros_odometry_local_;
  void                                          callbackOdometryLocal(mrs_lib::SubscribeHandler<nav_msgs::Odometry> &wrp);

  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_mavros_gps_;
  void                                              callbackNavsatFix(mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> &wrp);

  mrs_lib::SubscribeHandler<sensor_msgs::Range> sh_mavros_distance_sensor_;
  void                                          callbackDistanceSensor(mrs_lib::SubscribeHandler<sensor_msgs::Range> &wrp);

  mrs_lib::SubscribeHandler<sensor_msgs::Imu> sh_mavros_imu_;
  void                                        callbackImu(mrs_lib::SubscribeHandler<sensor_msgs::Imu> &wrp);

  mrs_lib::SubscribeHandler<std_msgs::Float64> sh_mavros_magnetometer_heading_;
  void                                         callbackMagnetometer(mrs_lib::SubscribeHandler<std_msgs::Float64> &wrp);

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_  = false;
  std::atomic<bool> armed_     = false;
  std::atomic<bool> connected_ = false;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void MrsUavPixhawkApi::initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers,
                                  [[maybe_unused]] const std::string &topic_prefix, [[maybe_unused]] const std::string &uav_name) {

  ros::NodeHandle nh_(parent_nh);

  common_handlers_ = common_handlers;

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MrsUavHwApi");

  param_loader.loadParam("services/mavros/command_long", _topic_mavros_command_long_);
  param_loader.loadParam("topics/mavros/state", _topic_mavros_state_);
  param_loader.loadParam("topics/mavros/odometry_local", _topic_mavros_odometry_local_);
  param_loader.loadParam("topics/mavros/gps", _topic_mavros_gps_);
  param_loader.loadParam("topics/mavros/distance_sensor", _topic_mavros_distance_sensor_);
  param_loader.loadParam("topics/mavros/imu", _topic_mavros_imu_);
  param_loader.loadParam("topics/mavros/magnetometer_heading", _topic_mavros_magnetometer_heading_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsUavHwDummyApi]: Could not load all parameters!");
    ros::shutdown();
  }

  // | --------------------- service clients -------------------- |

  sch_mavros_command_long_ = mrs_lib::ServiceClientHandler<mavros_msgs::CommandLong>(nh_, topic_prefix + "/" + _topic_mavros_command_long_);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MrsHwPixhawkApi";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_mavros_state_ = mrs_lib::SubscribeHandler<mavros_msgs::State>(shopts, topic_prefix + "/" + _topic_mavros_state_, ros::Duration(0.05),
                                                                   &MrsUavPixhawkApi::timeoutMavrosState, this, &MrsUavPixhawkApi::callbackMavrosState, this);

  sh_mavros_odometry_local_ =
      mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, topic_prefix + "/" + _topic_mavros_odometry_local_, &MrsUavPixhawkApi::callbackOdometryLocal, this);

  sh_mavros_gps_ =
      mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, topic_prefix + "/" + _topic_mavros_gps_, &MrsUavPixhawkApi::callbackNavsatFix, this);

  sh_mavros_distance_sensor_ = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, topic_prefix + "/" + _topic_mavros_distance_sensor_,
                                                                             &MrsUavPixhawkApi::callbackDistanceSensor, this);

  sh_mavros_imu_ = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, topic_prefix + "/" + _topic_mavros_imu_, &MrsUavPixhawkApi::callbackImu, this);

  sh_mavros_magnetometer_heading_ = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, topic_prefix + "/" + _topic_mavros_magnetometer_heading_,
                                                                                 &MrsUavPixhawkApi::callbackMagnetometer, this);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MrsUavHwDummyApi]: initialized");

  is_initialized_ = true;
}

//}

/* getDiagnostics() //{ */

mrs_msgs::HwApiDiagnostics MrsUavPixhawkApi::getDiagnostics() {

  mrs_msgs::HwApiDiagnostics diag;

  diag.stamp = ros::Time::now();

  diag.armed     = armed_;
  diag.offboard  = offboard_;
  diag.connected = connected_;

  return diag;
}

//}

/* getMode() //{ */

mrs_msgs::HwApiMode MrsUavPixhawkApi::getMode() {

  mrs_msgs::HwApiMode mode;

  mode.api_name = "PixhawkApi";
  mode.stamp    = ros::Time::now();

  mode.accepts_control_group_cmd = false;
  mode.accepts_actuator_cmd      = false;
  mode.accepts_attitude_rate_cmd = true;
  mode.accepts_attitude_cmd      = true;
  mode.accepts_translation_cmd   = false;

  mode.produces_range                = true;
  mode.produces_gnss                 = true;
  mode.produces_imu                  = true;
  mode.produces_altitude             = false;
  mode.produces_magnetometer_heading = true;
  mode.produces_odometry_local       = true;
  mode.produces_rc_channels          = true;

  return mode;
}

//}

/* callbackControlGroupCmd() //{ */

bool MrsUavPixhawkApi::callbackControlGroupCmd([[maybe_unused]] const mrs_msgs::HwApiControlGroupCmd &msg) {

  return false;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool MrsUavPixhawkApi::callbackAttitudeRateCmd([[maybe_unused]] const mrs_msgs::HwApiAttitudeRateCmd &msg) {

  return false;
}

//}

/* callbackAttitudeCmd() //{ */

bool MrsUavPixhawkApi::callbackAttitudeCmd([[maybe_unused]] const mrs_msgs::HwApiAttitudeCmd &msg) {

  return false;
}

//}

/* callbackTranslationCmd() //{ */

bool MrsUavPixhawkApi::callbackTranslationCmd([[maybe_unused]] const mrs_msgs::HwApiTranslationCmd &msg) {

  return false;
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> MrsUavPixhawkApi::callbackArming([[maybe_unused]] const bool &request) {

  std::stringstream ss;

  if (request) {

    ss << "Arming is not allowed using the companion computer.";
    ROS_WARN_STREAM_THROTTLE(1.0, "[PixhawkApi]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!offboard_) {

    ss << "can not disarm, not in OFFBOARD mode";
    ROS_WARN_STREAM_THROTTLE(1.0, "[PixhawkApi]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  mavros_msgs::CommandLong srv_out;

  srv_out.request.broadcast    = false;
  srv_out.request.command      = 400;  // the code for arming
  srv_out.request.confirmation = true;

  srv_out.request.param1 = request ? 1 : 0;      // arm or disarm?
  srv_out.request.param2 = request ? 0 : 21196;  // 21196 allows to disarm even in mid-flight
  srv_out.request.param3 = 0;
  srv_out.request.param4 = 0;
  srv_out.request.param5 = 0;
  srv_out.request.param6 = 0;
  srv_out.request.param7 = 0;

  ROS_INFO("[PixhawkApi]: calling for %s", request ? "arming" : "disarming");

  if (sch_mavros_command_long_.call(srv_out)) {

    if (srv_out.response.success) {

      ss << "service call for " << (request ? "arming" : "disarming") << " was successful";
      ROS_INFO_STREAM_THROTTLE(1.0, "[PixhawkApi]: " << ss.str());

    } else {
      ss << "service call for " << (request ? "arming" : "disarming") << " failed";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[PixhawkApi]: " << ss.str());
    }

  } else {
    ss << "calling for " << (request ? "arming" : "disarming") << " resulted in failure: '" << srv_out.response.result << "'";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[PixhawkApi]: " << ss.str());
  }

  return {srv_out.response.success, ss.str()};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> MrsUavPixhawkApi::callbackOffboard([[maybe_unused]] const bool &request) {

  return {false, "Dummy interface does not allow to switch to offboard."};
}

//}

// | ------------------- additional methods ------------------- |

/* timeoutMavrosState() //{ */

void MrsUavPixhawkApi::timeoutMavrosState([[maybe_unused]] const std::string &topic, const ros::Time &last_msg, [[maybe_unused]] const int n_pubs) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_mavros_state_.hasMsg()) {
    return;
  }

  ros::Duration time = ros::Time::now() - last_msg;

  ROS_ERROR_THROTTLE(1.0, "[MrsUavPixhawkApi]: Not recieving Mavros state message for '%.3f s'! Setup the PixHawk SD card!!", time.toSec());
  ROS_INFO_THROTTLE(1.0, "[MrsUavPixhawkApi]: This could be also caused by the not being PixHawk booted properly due to, e.g., antispark connector jerkyness.");
  ROS_INFO_THROTTLE(1.0, "[MrsUavPixhawkApi]: The Mavros state should be supplied at 100 Hz to provided fast refresh rate on the state of the OFFBOARD mode.");
  ROS_INFO_THROTTLE(1.0, "[MrsUavPixhawkApi]: If missing, the UAV could be disarmed by safety routines while not knowing it has switched to the MANUAL mode.");
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackMavrosState() */

void MrsUavPixhawkApi::callbackMavrosState(mrs_lib::SubscribeHandler<mavros_msgs::State> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPixhawkApi]: getting Mavros state");

  mavros_msgs::StateConstPtr state = wrp.getMsg();

  offboard_  = state->mode == "OFFBOARD";
  armed_     = state->armed;
  connected_ = true;

  // | ----------------- publish the diagnostics ---------------- |

  mrs_msgs::HwApiDiagnostics diag;

  diag.stamp     = ros::Time::now();
  diag.armed     = armed_;
  diag.offboard  = offboard_;
  diag.connected = connected_;

  common_handlers_->publishers.publishDiagnostics(diag);
}

//}

/* callbackOdometryLocal() //{ */

void MrsUavPixhawkApi::callbackOdometryLocal(mrs_lib::SubscribeHandler<nav_msgs::Odometry> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPixhawkApi]: getting local odometry");

  nav_msgs::OdometryConstPtr odom = wrp.getMsg();

  common_handlers_->publishers.publishOdometryLocal(*odom);
}

//}

/* callbackNavsatFix() //{ */

void MrsUavPixhawkApi::callbackNavsatFix(mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPixhawkApi]: getting NavSat fix");

  sensor_msgs::NavSatFixConstPtr gnss = wrp.getMsg();

  common_handlers_->publishers.publishGNSS(*gnss);
}

//}

/* callbackDistanceSensor() //{ */

void MrsUavPixhawkApi::callbackDistanceSensor(mrs_lib::SubscribeHandler<sensor_msgs::Range> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPixhawkApi]: getting distnace sensor");

  sensor_msgs::RangeConstPtr range = wrp.getMsg();

  common_handlers_->publishers.publishRange(*range);
}

//}

/* callbackImu() //{ */

void MrsUavPixhawkApi::callbackImu(mrs_lib::SubscribeHandler<sensor_msgs::Imu> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPixhawkApi]: getting IMU");

  sensor_msgs::ImuConstPtr imu = wrp.getMsg();

  common_handlers_->publishers.publishIMU(*imu);
}

//}

/* callbackCompass() //{ */

void MrsUavPixhawkApi::callbackMagnetometer(mrs_lib::SubscribeHandler<std_msgs::Float64> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPixhawkApi]: getting magnetometer heading");

  std_msgs::Float64ConstPtr mag = wrp.getMsg();

  mrs_msgs::Float64Stamped mag_out;
  mag_out.header.stamp = ros::Time::now();
  mag_out.value        = mag->data;

  common_handlers_->publishers.publishMagnetometerHeading(mag_out);
}

//}

}  // namespace mrs_uav_pixhawk_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_pixhawk_api::MrsUavPixhawkApi, mrs_uav_hw_api::MrsUavHwApi)
