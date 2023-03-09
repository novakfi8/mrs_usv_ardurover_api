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

#include <geometry_msgs/QuaternionStamped.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/ActuatorControl.h>

//}

/* defines //{ */

#define PWM_MIDDLE 1500
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_DEADBAND 200
#define PWM_RANGE PWM_MAX - PWM_MIN

//}

namespace mrs_uav_px4_api
{

/* class MrsUavPx4Api //{ */

class MrsUavPx4Api : public mrs_uav_hw_api::MrsUavHwApi {

public:
  ~MrsUavPx4Api(){};

  void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers, const std::string &topic_prefix,
                  const std::string &uav_name);

  // | --------------------- status methods --------------------- |

  mrs_msgs::HwApiStatus       getStatus();
  mrs_msgs::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd> &wrp);
  bool callbackControlGroupCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd> &wrp);
  bool callbackAttitudeRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd> &wrp);
  bool callbackAttitudeCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd> &wrp);
  bool callbackAccelerationHdgRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgRateCmd> &wrp);
  bool callbackAccelerationHdgCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgCmd> &wrp);
  bool callbackVelocityHdgRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgRateCmd> &wrp);
  bool callbackVelocityHdgCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgCmd> &wrp);
  bool callbackPositionCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd> &wrp);

  void callbackTrackerCmd(mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand> &wrp);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- parameters ----------------------- |

  mrs_msgs::HwApiCapabilities _capabilities_;

  std::string _service_mavros_command_long_;
  std::string _service_mavros_mode_;

  std::string _topic_mavros_state_;
  std::string _topic_mavros_odometry_local_;
  std::string _topic_mavros_gps_;
  std::string _topic_mavros_distance_sensor_;
  std::string _topic_mavros_imu_;
  std::string _topic_mavros_magnetometer_heading_;
  std::string _topic_mavros_attitude_target_;
  std::string _topic_mavros_actuator_control;
  std::string _topic_mavros_rc_;
  std::string _topic_mavros_altitude_;
  std::string _topic_mavros_battery_;

  double _mavros_timeout_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mavros_msgs::CommandLong> sch_mavros_command_long_;
  mrs_lib::ServiceClientHandler<mavros_msgs::SetMode>     sch_mavros_mode_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mavros_msgs::State> sh_mavros_state_;

  void   timeoutMavrosState(const std::string &topic, const ros::Time &last_msg, const int n_pubs);
  double RCChannelToRange(const double &rc_value);
  void   callbackMavrosState(mrs_lib::SubscribeHandler<mavros_msgs::State> &wrp);

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

  mrs_lib::SubscribeHandler<mavros_msgs::RCIn> sh_mavros_rc_;
  void                                         callbackRC(mrs_lib::SubscribeHandler<mavros_msgs::RCIn> &wrp);

  mrs_lib::SubscribeHandler<mavros_msgs::Altitude> sh_mavros_altitude_;
  void                                             callbackAltitude(mrs_lib::SubscribeHandler<mavros_msgs::Altitude> &wrp);

  mrs_lib::SubscribeHandler<sensor_msgs::BatteryState> sh_mavros_battery_;
  void                                                 callbackBattery(mrs_lib::SubscribeHandler<sensor_msgs::BatteryState> &wrp);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mavros_msgs::AttitudeTarget>  ph_mavros_attitude_target_;
  mrs_lib::PublisherHandler<mavros_msgs::ActuatorControl> ph_mavros_actuator_control_;

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_ = false;
  std::string       mode_;
  std::atomic<bool> armed_     = false;
  std::atomic<bool> connected_ = false;
  std::mutex        mutex_status_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void MrsUavPx4Api::initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers,
                              [[maybe_unused]] const std::string &topic_prefix, [[maybe_unused]] const std::string &uav_name) {

  ros::NodeHandle nh_(parent_nh);

  common_handlers_ = common_handlers;

  _capabilities_.api_name = "Px4Api";

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MrsUavHwApi");

  param_loader.loadParam("mavros_timeout", _mavros_timeout_);

  param_loader.loadParam("inputs/control_group", (bool &)_capabilities_.accepts_control_group_cmd);
  param_loader.loadParam("inputs/attitude_rate", (bool &)_capabilities_.accepts_attitude_rate_cmd);
  param_loader.loadParam("inputs/attitude", (bool &)_capabilities_.accepts_attitude_cmd);

  param_loader.loadParam("outputs/distance_sensor", (bool &)_capabilities_.produces_distance_sensor);
  param_loader.loadParam("outputs/gnss", (bool &)_capabilities_.produces_gnss);
  param_loader.loadParam("outputs/imu", (bool &)_capabilities_.produces_imu);
  param_loader.loadParam("outputs/altitude", (bool &)_capabilities_.produces_altitude);
  param_loader.loadParam("outputs/magnetometer_heading", (bool &)_capabilities_.produces_magnetometer_heading);
  param_loader.loadParam("outputs/rc_channels", (bool &)_capabilities_.produces_rc_channels);
  param_loader.loadParam("outputs/battery_state", (bool &)_capabilities_.produces_battery_state);
  param_loader.loadParam("outputs/position", (bool &)_capabilities_.produces_position);
  param_loader.loadParam("outputs/orientation", (bool &)_capabilities_.produces_orientation);
  param_loader.loadParam("outputs/velocity", (bool &)_capabilities_.produces_velocity);
  param_loader.loadParam("outputs/angular_velocity", (bool &)_capabilities_.produces_angular_velocity);
  param_loader.loadParam("outputs/odometry", (bool &)_capabilities_.produces_odometry);

  param_loader.loadParam("services/mavros/command_long", _service_mavros_command_long_);
  param_loader.loadParam("services/mavros/mode", _service_mavros_mode_);
  param_loader.loadParam("topics/mavros/state", _topic_mavros_state_);
  param_loader.loadParam("topics/mavros/odometry_local", _topic_mavros_odometry_local_);
  param_loader.loadParam("topics/mavros/gps", _topic_mavros_gps_);
  param_loader.loadParam("topics/mavros/distance_sensor", _topic_mavros_distance_sensor_);
  param_loader.loadParam("topics/mavros/imu", _topic_mavros_imu_);
  param_loader.loadParam("topics/mavros/magnetometer_heading", _topic_mavros_magnetometer_heading_);
  param_loader.loadParam("topics/mavros/attitude_target", _topic_mavros_attitude_target_);
  param_loader.loadParam("topics/mavros/actuator_control", _topic_mavros_actuator_control);
  param_loader.loadParam("topics/mavros/rc", _topic_mavros_rc_);
  param_loader.loadParam("topics/mavros/altitude", _topic_mavros_altitude_);
  param_loader.loadParam("topics/mavros/battery", _topic_mavros_battery_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsUavTelloApi]: Could not load all parameters!");
    ros::shutdown();
  }

  // | --------------------- service clients -------------------- |

  sch_mavros_command_long_ = mrs_lib::ServiceClientHandler<mavros_msgs::CommandLong>(nh_, topic_prefix + "/" + _service_mavros_command_long_);
  sch_mavros_mode_         = mrs_lib::ServiceClientHandler<mavros_msgs::SetMode>(nh_, topic_prefix + "/" + _service_mavros_mode_);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MrsHwPx4Api";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_mavros_state_ = mrs_lib::SubscribeHandler<mavros_msgs::State>(shopts, topic_prefix + "/" + _topic_mavros_state_, ros::Duration(0.05),
                                                                   &MrsUavPx4Api::timeoutMavrosState, this, &MrsUavPx4Api::callbackMavrosState, this);

  sh_mavros_odometry_local_ =
      mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, topic_prefix + "/" + _topic_mavros_odometry_local_, &MrsUavPx4Api::callbackOdometryLocal, this);

  sh_mavros_gps_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, topic_prefix + "/" + _topic_mavros_gps_, &MrsUavPx4Api::callbackNavsatFix, this);

  sh_mavros_distance_sensor_ =
      mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, topic_prefix + "/" + _topic_mavros_distance_sensor_, &MrsUavPx4Api::callbackDistanceSensor, this);

  sh_mavros_imu_ = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, topic_prefix + "/" + _topic_mavros_imu_, &MrsUavPx4Api::callbackImu, this);

  sh_mavros_magnetometer_heading_ =
      mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, topic_prefix + "/" + _topic_mavros_magnetometer_heading_, &MrsUavPx4Api::callbackMagnetometer, this);

  sh_mavros_rc_ = mrs_lib::SubscribeHandler<mavros_msgs::RCIn>(shopts, topic_prefix + "/" + _topic_mavros_rc_, &MrsUavPx4Api::callbackRC, this);

  sh_mavros_altitude_ =
      mrs_lib::SubscribeHandler<mavros_msgs::Altitude>(shopts, topic_prefix + "/" + _topic_mavros_altitude_, &MrsUavPx4Api::callbackAltitude, this);

  sh_mavros_battery_ =
      mrs_lib::SubscribeHandler<sensor_msgs::BatteryState>(shopts, topic_prefix + "/" + _topic_mavros_battery_, &MrsUavPx4Api::callbackBattery, this);

  // | ----------------------- publishers ----------------------- |

  ph_mavros_attitude_target_  = mrs_lib::PublisherHandler<mavros_msgs::AttitudeTarget>(nh_, topic_prefix + "/" + _topic_mavros_attitude_target_, 1);
  ph_mavros_actuator_control_ = mrs_lib::PublisherHandler<mavros_msgs::ActuatorControl>(nh_, topic_prefix + "/" + _topic_mavros_actuator_control, 1);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MrsUavTelloApi]: initialized");

  is_initialized_ = true;
}

//}

/* getStatus() //{ */

mrs_msgs::HwApiStatus MrsUavPx4Api::getStatus() {

  mrs_msgs::HwApiStatus status;

  status.stamp = ros::Time::now();

  {
    std::scoped_lock lock(mutex_status_);

    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  return status;
}

//}

/* getCapabilities() //{ */

mrs_msgs::HwApiCapabilities MrsUavPx4Api::getCapabilities() {

  _capabilities_.stamp = ros::Time::now();

  return _capabilities_;
}

//}

/* callbackControlActuatorCmd() //{ */

bool MrsUavPx4Api::callbackActuatorCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd> &wrp) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting actuator cmd");

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool MrsUavPx4Api::callbackControlGroupCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd> &wrp) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting control group cmd");

  if (!_capabilities_.accepts_control_group_cmd) {
    ROS_ERROR("[MrsUavPx4Api]: the control group input is not enabled in the config file");
    return false;
  }

  auto msg = wrp.getMsg();

  mavros_msgs::ActuatorControl msg_out;

  msg_out.header.frame_id = "base_link";
  msg_out.header.stamp    = msg->stamp;

  msg_out.controls[0] = msg->roll;
  msg_out.controls[1] = -msg->pitch;
  msg_out.controls[2] = -msg->yaw;
  msg_out.controls[3] = msg->throttle;

  ph_mavros_actuator_control_.publish(msg_out);

  return true;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool MrsUavPx4Api::callbackAttitudeRateCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd> &wrp) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting attitude rate cmd");

  if (!_capabilities_.accepts_attitude_rate_cmd) {
    ROS_ERROR_THROTTLE(1.0, "[MrsUavPx4Api]: attitude rate input is not enabled in the config file");
    return false;
  }

  auto msg = wrp.getMsg();

  mavros_msgs::AttitudeTarget attitude_target;

  attitude_target.header.frame_id = "base_link";
  attitude_target.header.stamp    = msg->stamp;

  attitude_target.body_rate = msg->body_rate;
  attitude_target.thrust    = msg->throttle;

  attitude_target.type_mask = attitude_target.IGNORE_ATTITUDE;

  ph_mavros_attitude_target_.publish(attitude_target);

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool MrsUavPx4Api::callbackAttitudeCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd> &wrp) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting attitude cmd");

  if (!_capabilities_.accepts_attitude_cmd) {
    ROS_ERROR_THROTTLE(1.0, "[MrsUavPx4Api]: attitude input is not enabled in the config file");
    return false;
  }

  auto msg = wrp.getMsg();

  mavros_msgs::AttitudeTarget attitude_target;

  attitude_target.header.frame_id = "base_link";
  attitude_target.header.stamp    = msg->stamp;

  attitude_target.orientation.x = msg->orientation.x;
  attitude_target.orientation.y = msg->orientation.y;
  attitude_target.orientation.z = msg->orientation.z;
  attitude_target.orientation.w = msg->orientation.w;

  attitude_target.thrust = msg->throttle;

  attitude_target.type_mask = attitude_target.IGNORE_YAW_RATE | attitude_target.IGNORE_ROLL_RATE | attitude_target.IGNORE_PITCH_RATE;

  ph_mavros_attitude_target_.publish(attitude_target);

  return true;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool MrsUavPx4Api::callbackAccelerationHdgRateCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgRateCmd> &wrp) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting acceleration+hdg rate cmd");

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool MrsUavPx4Api::callbackAccelerationHdgCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgCmd> &wrp) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting acceleration+hdg cmd");

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool MrsUavPx4Api::callbackVelocityHdgRateCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgRateCmd> &wrp) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting velocity+hdg rate cmd");

  return false;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool MrsUavPx4Api::callbackVelocityHdgCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgCmd> &wrp) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting velocity+hdg  cmd");

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool MrsUavPx4Api::callbackPositionCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd> &wrp) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting position cmd");

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void MrsUavPx4Api::callbackTrackerCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand> &wrp) {
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> MrsUavPx4Api::callbackArming([[maybe_unused]] const bool &request) {

  std::stringstream ss;

  /* if (request) { */

  /*   ss << "Arming is not allowed using the companion computer."; */
  /*   ROS_WARN_STREAM_THROTTLE(1.0, "[Px4Api]: " << ss.str()); */
  /*   return std::tuple(false, ss.str()); */
  /* } */

  if (!request && !offboard_) {

    ss << "can not disarm, not in OFFBOARD mode";
    ROS_WARN_STREAM_THROTTLE(1.0, "[Px4Api]: " << ss.str());
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

  ROS_INFO("[Px4Api]: calling for %s", request ? "arming" : "disarming");

  if (sch_mavros_command_long_.call(srv_out)) {

    if (srv_out.response.success) {

      ss << "service call for " << (request ? "arming" : "disarming") << " was successful";
      ROS_INFO_STREAM_THROTTLE(1.0, "[Px4Api]: " << ss.str());

    } else {
      ss << "service call for " << (request ? "arming" : "disarming") << " failed";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[Px4Api]: " << ss.str());
    }

  } else {
    ss << "calling for " << (request ? "arming" : "disarming") << " resulted in failure: '" << srv_out.response.result << "'";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[Px4Api]: " << ss.str());
  }

  return {srv_out.response.success, ss.str()};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> MrsUavPx4Api::callbackOffboard(void) {

  mavros_msgs::SetMode srv;

  srv.request.base_mode   = 0;
  srv.request.custom_mode = "OFFBOARD";

  bool res = sch_mavros_mode_.call(srv);

  std::stringstream ss;

  if (!res) {

    ss << "Service call for offboard failed!";

    ROS_ERROR_THROTTLE(1.0, "[Px4Api]: %s", ss.str().c_str());
    return {false, ss.str()};

  } else {

    if (srv.response.mode_sent != 1) {

      ss << "service call for offboard failed, returned " << srv.response.mode_sent;

      ROS_WARN_THROTTLE(1.0, "[Px4Api]: %s", ss.str().c_str());

      return {false, ss.str()};

    } else {

      ss << "switched to offboard mode";

      return {true, ss.str()};
    }
  }
}

//}

// | ------------------- additional methods ------------------- |

/* timeoutMavrosState() //{ */

void MrsUavPx4Api::timeoutMavrosState([[maybe_unused]] const std::string &topic, const ros::Time &last_msg, [[maybe_unused]] const int n_pubs) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_mavros_state_.hasMsg()) {
    return;
  }

  ros::Duration time = ros::Time::now() - last_msg;

  if (time.toSec() > _mavros_timeout_) {

    {
      std::scoped_lock lock(mutex_status_);

      connected_ = false;
      offboard_  = false;
      armed_     = false;
      mode_      = "";
    }

    ROS_ERROR_THROTTLE(1.0, "[MrsUavPx4Api]: Have not received Mavros state for more than '%.3f s'", time.toSec());

  } else {

    ROS_ERROR_THROTTLE(1.0, "[MrsUavPx4Api]: Not recieving Mavros state message for '%.3f s'! Setup the PixHawk SD card!!", time.toSec());
    ROS_INFO_THROTTLE(1.0, "[MrsUavPx4Api]: This could be also caused by the not being PixHawk booted properly due to, e.g., antispark connector jerkyness.");
    ROS_INFO_THROTTLE(1.0, "[MrsUavPx4Api]: The Mavros state should be supplied at 100 Hz to provided fast refresh rate on the state of the OFFBOARD mode.");
    ROS_INFO_THROTTLE(1.0, "[MrsUavPx4Api]: If missing, the UAV could be disarmed by safety routines while not knowing it has switched to the MANUAL mode.");
  }
}

//}

/* RCChannelToRange() //{ */

double MrsUavPx4Api::RCChannelToRange(const double &rc_value) {

  double tmp_0_to_1 = (rc_value - double(PWM_MIN)) / (double(PWM_RANGE));

  if (tmp_0_to_1 > 1.0) {
    tmp_0_to_1 = 1.0;
  } else if (tmp_0_to_1 < 0.0) {
    tmp_0_to_1 = 0.0;
  }

  return tmp_0_to_1;
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackMavrosState() */

void MrsUavPx4Api::callbackMavrosState(mrs_lib::SubscribeHandler<mavros_msgs::State> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting Mavros state");

  mavros_msgs::StateConstPtr state = wrp.getMsg();

  {
    std::scoped_lock lock(mutex_status_);

    offboard_  = state->mode == "OFFBOARD";
    armed_     = state->armed;
    connected_ = true;
    mode_      = state->mode;
  }

  // | ----------------- publish the diagnostics ---------------- |

  mrs_msgs::HwApiStatus status;

  {
    std::scoped_lock lock(mutex_status_);

    status.stamp     = ros::Time::now();
    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  common_handlers_->publishers.publishStatus(status);
}

//}

/* callbackOdometryLocal() //{ */

void MrsUavPx4Api::callbackOdometryLocal(mrs_lib::SubscribeHandler<nav_msgs::Odometry> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting Mavros's local odometry");

  nav_msgs::OdometryConstPtr odom = wrp.getMsg();

  // | -------------------- publish position -------------------- |

  if (_capabilities_.produces_position) {

    geometry_msgs::PointStamped position;

    position.header = odom->header;
    position.point  = odom->pose.pose.position;

    common_handlers_->publishers.publishPosition(position);
  }

  // | ------------------- publish orientation ------------------ |

  if (_capabilities_.produces_orientation) {

    geometry_msgs::QuaternionStamped orientation;

    orientation.header     = odom->header;
    orientation.quaternion = odom->pose.pose.orientation;

    common_handlers_->publishers.publishOrientation(orientation);
  }

  // | -------------------- publish velocity -------------------- |

  if (_capabilities_.produces_velocity) {

    geometry_msgs::Vector3Stamped velocity;

    velocity.header.stamp    = odom->header.stamp;
    velocity.header.frame_id = odom->child_frame_id;
    velocity.vector          = odom->twist.twist.linear;

    common_handlers_->publishers.publishVelocity(velocity);
  }

  // | ---------------- publish angular velocity ---------------- |

  if (_capabilities_.produces_angular_velocity) {

    geometry_msgs::Vector3Stamped angular_velocity;

    angular_velocity.header.stamp    = odom->header.stamp;
    angular_velocity.header.frame_id = odom->child_frame_id;
    angular_velocity.vector          = odom->twist.twist.angular;

    common_handlers_->publishers.publishAngularVelocity(angular_velocity);
  }

  // | -------------------- publish odometry -------------------- |

  if (_capabilities_.produces_odometry) {
    common_handlers_->publishers.publishOdometry(*odom);
  }
}

//}

/* callbackNavsatFix() //{ */

void MrsUavPx4Api::callbackNavsatFix(mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting NavSat fix");

  if (_capabilities_.produces_gnss) {

    sensor_msgs::NavSatFixConstPtr gnss = wrp.getMsg();

    common_handlers_->publishers.publishGNSS(*gnss);
  }
}

//}

/* callbackDistanceSensor() //{ */

void MrsUavPx4Api::callbackDistanceSensor(mrs_lib::SubscribeHandler<sensor_msgs::Range> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting distnace sensor");

  if (_capabilities_.produces_distance_sensor) {

    sensor_msgs::RangeConstPtr range = wrp.getMsg();

    common_handlers_->publishers.publishDistanceSensor(*range);
  }
}

//}

/* callbackImu() //{ */

void MrsUavPx4Api::callbackImu(mrs_lib::SubscribeHandler<sensor_msgs::Imu> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting IMU");

  if (_capabilities_.produces_imu) {

    sensor_msgs::ImuConstPtr imu = wrp.getMsg();

    common_handlers_->publishers.publishIMU(*imu);
  }
}

//}

/* callbackCompass() //{ */

void MrsUavPx4Api::callbackMagnetometer(mrs_lib::SubscribeHandler<std_msgs::Float64> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting magnetometer heading");

  if (_capabilities_.produces_magnetometer_heading) {

    std_msgs::Float64ConstPtr mag = wrp.getMsg();

    mrs_msgs::Float64Stamped mag_out;
    mag_out.header.stamp = ros::Time::now();
    mag_out.value        = mag->data;

    common_handlers_->publishers.publishMagnetometerHeading(mag_out);
  }
}

//}

/* callbackRC() //{ */

void MrsUavPx4Api::callbackRC(mrs_lib::SubscribeHandler<mavros_msgs::RCIn> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting RC");

  if (_capabilities_.produces_rc_channels) {

    mavros_msgs::RCInConstPtr msg_in = wrp.getMsg();

    mrs_msgs::HwApiRcChannels rc_out;

    rc_out.stamp = msg_in->header.stamp;

    for (int i = 0; i < msg_in->channels.size(); i++) {
      rc_out.channels.push_back(RCChannelToRange(msg_in->channels[i]));
    }

    common_handlers_->publishers.publishRcChannels(rc_out);
  }
}

//}

/* callbackAltitude() //{ */

void MrsUavPx4Api::callbackAltitude(mrs_lib::SubscribeHandler<mavros_msgs::Altitude> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting Altitude");

  if (_capabilities_.produces_altitude) {

    mavros_msgs::AltitudeConstPtr altitude_in = wrp.getMsg();

    mrs_msgs::HwApiAltitude altitude_out;

    altitude_out.stamp = altitude_in->header.stamp;
    altitude_out.amsl  = altitude_in->amsl;

    common_handlers_->publishers.publishAltitude(altitude_out);
  }
}

//}

/* callbackBattery() //{ */

void MrsUavPx4Api::callbackBattery(mrs_lib::SubscribeHandler<sensor_msgs::BatteryState> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting battery");

  if (_capabilities_.produces_battery_state) {

    sensor_msgs::BatteryStateConstPtr msg = wrp.getMsg();

    common_handlers_->publishers.publishBatteryState(*msg);
  }
}

//}

}  // namespace mrs_uav_px4_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_px4_api::MrsUavPx4Api, mrs_uav_hw_api::MrsUavHwApi)
