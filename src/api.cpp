/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_hw_api/api.h>

#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/set_bool.hpp>

/* #include <mrs_modules_msgs/Bestpos.h> */

#include <nav_msgs/msg/odometry.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/errorgraph/error_publisher.h>
#include <mrs_lib/transformer.h>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/msg/actuator_control.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <mavros_msgs/msg/position_target.hpp>

//}

/* defines //{ */

#define PWM_MIDDLE 1500
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_DEADBAND 200
#define PWM_RANGE PWM_MAX - PWM_MIN
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0)

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_usv_ardurover_api
{

/* class MrsUsvArduroverApi //{ */

class MrsUsvArduroverApi : public mrs_uav_hw_api::MrsUavHwApi {

public:
  ~MrsUsvArduroverApi(){};

  void initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);

  void destroy();
  
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;

  // | --------------------- status methods --------------------- |

  mrs_msgs::msg::HwApiStatus       getStatus();
  mrs_msgs::msg::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg);
  bool callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
  bool callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg);
  bool callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgCmd(const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgCmd(const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg);
  bool callbackPositionCmd(const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg);

  void callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  bool is_initialized_ = false;
 
  // | -------------------- errorgraph ------------------- |
  enum class error_type_t : uint16_t
  {
    not_receiving_mavros_state,
    not_receiving_ground_truth,
    not_receiving_rtk,
    not_receiving_gps,
    not_receiving_distance_sensor,
    not_receiving_imu,
  };

  std::shared_ptr<mrs_lib::errorgraph::ErrorPublisher> error_publisher_;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  rclcpp::Time last_mavros_state_time_;
  std::mutex   mutex_last_mavros_state_time_;

  // | ----------------------- parameters ----------------------- |

  mrs_msgs::msg::HwApiCapabilities _capabilities_;

  std::string _uav_name_;
  std::string _body_frame_name_;
  std::string _world_frame_name_;

  double _mavros_timeout_;
  // Needs a default value
  rclcpp::Duration _general_topic_timeout_ = rclcpp::Duration::from_seconds(1);
  double           _mavros_passable_delay_;

  bool _simulation_;

  double      _sim_rtk_utm_x_;
  double      _sim_rtk_utm_y_;
  std::string _sim_rtk_utm_zone_;
  double      _sim_rtk_amsl_;

  double _ref_sin_lat;
  double _ref_cos_lat;
  double _ref_lat;
  double _ref_lon;
  double _ref_utm_x;
  double _ref_utm_y;
  bool   _ref_latlon_init = false;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mavros_msgs::srv::CommandLong> sch_mavros_command_long_;
  mrs_lib::ServiceClientHandler<mavros_msgs::srv::SetMode>     sch_mavros_mode_;
  
  // | --------------------- service servers -------------------- |
  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool> sh_ignore_ground_truth_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>         sh_ground_truth_;
  mrs_lib::SubscriberHandler<mavros_msgs::msg::State>         sh_mavros_state_;
  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>         sh_mavros_odometry_local_;
  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>         sh_mavros_odometry_in_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>     sh_mavros_gps_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>         sh_mavros_distance_sensor_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>           sh_mavros_imu_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>          sh_mavros_magnetometer_heading_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField> sh_mavros_magnetic_field_;
  mrs_lib::SubscriberHandler<mavros_msgs::msg::RCIn>          sh_mavros_rc_;
  mrs_lib::SubscriberHandler<mavros_msgs::msg::Altitude>      sh_mavros_altitude_;
  mrs_lib::SubscriberHandler<mavros_msgs::msg::GPSRAW>        sh_gps_status_raw_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState>  sh_mavros_battery_;
  /* mrs_lib::SubscriberHandler<mrs_modules_msgs::msg::Bestpos> sh_rtk_; */

  void callbackGroundTruth(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackMavrosState(const mavros_msgs::msg::State::ConstSharedPtr msg);
  void callbackOdometryLocal(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackOdometryIn(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackNavsatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void callbackDistanceSensor(const sensor_msgs::msg::Range::ConstSharedPtr msg);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void callbackMagnetometer(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void callbackMagneticField(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);
  void callbackRC(const mavros_msgs::msg::RCIn::ConstSharedPtr msg);
  void callbackAltitude(const mavros_msgs::msg::Altitude::ConstSharedPtr msg);
  void callbackGpsStatusRaw(const mavros_msgs::msg::GPSRAW::ConstSharedPtr msg);
  void callbackBattery(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
  bool callbackIgnoreGroundTruth(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
  /* void callbackRTK(const mrs_modules_msgs::msg::Bestpos::ConstSharedPtr msg); */
  
  void timeoutMavrosState(void);
  void timeoutGeneralTopic(const std::string &topic_name, const rclcpp::Time &last_msg_time, const error_type_t &error_type,
                           const std::string &error_description);

  double RCChannelToRange(const double &rc_value);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mavros_msgs::msg::AttitudeTarget>  ph_mavros_attitude_target_;
  mrs_lib::PublisherHandler<mavros_msgs::msg::ActuatorControl> ph_mavros_actuator_control_;
  
  mrs_lib::PublisherHandler<mavros_msgs::msg::PositionTarget>  ph_mavros_position_target_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::Twist>         ph_mavros_velocity_target_;
  
  mrs_lib::PublisherHandler<std_msgs::msg::Float32> pub_left_usv_thrust_cmd_;
  mrs_lib::PublisherHandler<std_msgs::msg::Float32> pub_right_usv_thrust_cmd_;
  mrs_lib::PublisherHandler<std_msgs::msg::Float32> pub_left_usv_thrust_angle_;
  mrs_lib::PublisherHandler<std_msgs::msg::Float32> pub_right_usv_thrust_angle_;
  
  // | ------------------------- timers ------------------------- |

  std::shared_ptr<TimerType> timer_main_;
  std::shared_ptr<TimerType> timer_topic_check_;

  void timerMain();

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_ = false;
  std::string       mode_;
  std::atomic<bool> armed_     = false;
  std::atomic<bool> connected_ = false;
  std::mutex        mutex_status_;
  
  std::mutex                     mutex_orientation_;
  geometry_msgs::msg::Quaternion orientation_;
  std::mutex                     mutex_angular_velocity_;
  geometry_msgs::msg::Vector3    angular_velocity_;
    
  mrs_lib::Transformer transformer_;
  std::mutex           transformer_mutex;
  
  bool position_cmd_map = false;
  std::string       node_name = "MrsUsvArduroverApi";

  template <class SubscriberHandler_T, typename CbkMsg_T, typename CbkTim_T>
  static void init_subscriber_handler(MrsUsvArduroverApi *this_ptr, SubscriberHandler_T &subscriber_handler, const mrs_lib::SubscriberHandlerOptions &options,
                                      const std::string &topic, CbkMsg_T cbk_msg, CbkTim_T cbk_tim, error_type_t error_type, const std::string &error_msg) {
    const typename SubscriberHandler_T::message_callback_t message_callback = [this_ptr, cbk_msg](const auto &msg) { (this_ptr->*cbk_msg)(msg); };

    const typename SubscriberHandler_T::timeout_callback_t timeout_callback = [this_ptr, cbk_tim, error_type, error_msg](const auto &topic_name,
                                                                                                                         const auto &last_message) {
      (this_ptr->*cbk_tim)(topic_name, last_message, error_type, error_msg);
    };

    subscriber_handler = SubscriberHandler_T(options, topic, timeout_callback, message_callback);
  }


};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void MrsUsvArduroverApi::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) {

  node_            = node;
  clock_           = node_->get_clock();
  common_handlers_ = common_handlers;

  _uav_name_         = common_handlers->getUavName();
  _body_frame_name_  = common_handlers->getBodyFrameName();
  _world_frame_name_ = common_handlers->getWorldFrameName();

  _capabilities_.api_name = "ArduroverApi";
  
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_      = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  last_mavros_state_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  error_publisher_ = std::make_shared<mrs_lib::errorgraph::ErrorPublisher>(node_, clock_, "HwApiManager", "ArduroverApi");

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader local_param_loader(node_, "MrsUsvArduroverApi");
  
  std::vector<std::string> config_files;
  common_handlers_->main_param_loader->loadParamReusable("configs", config_files);
  
  common_handlers_->main_param_loader->loadParam("simulation", _simulation_);
  
  std::string custom_config_file = "";
  common_handlers_->main_param_loader->loadParam("custom_config", custom_config_file);
  if (custom_config_file != "") {
    local_param_loader.addYamlFile(custom_config_file);
  }
  
  if (!common_handlers_->main_param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    error_publisher_->addOneshotError("Could not load all parameters!");
    error_publisher_->flushAndShutdown();
  }

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "loading config file '%s'", config_file.c_str());
    local_param_loader.addYamlFile(config_file);
  }
    
  local_param_loader.loadParam("mavros_timeout", _mavros_timeout_);
  local_param_loader.loadParam("mavros_passable_delay", _mavros_passable_delay_);
  
  local_param_loader.loadParam("gnss/utm_x", _sim_rtk_utm_x_);
  local_param_loader.loadParam("gnss/utm_y", _sim_rtk_utm_y_);
  local_param_loader.loadParam("gnss/utm_zone", _sim_rtk_utm_zone_);
  local_param_loader.loadParam("gnss/amsl", _sim_rtk_amsl_);

  local_param_loader.loadParam("input_mode/control_group", (bool &)_capabilities_.accepts_control_group_cmd);
  local_param_loader.loadParam("input_mode/attitude_rate", (bool &)_capabilities_.accepts_attitude_rate_cmd);
  local_param_loader.loadParam("input_mode/attitude", (bool &)_capabilities_.accepts_attitude_cmd);
  local_param_loader.loadParam("input_mode/position", (bool &)_capabilities_.accepts_position_cmd);
  local_param_loader.loadParam("input_mode/velocity_hdg_rate", (bool &)_capabilities_.accepts_velocity_hdg_rate_cmd);
  local_param_loader.loadParam("input_mode/actuator", (bool &)_capabilities_.accepts_actuator_cmd);

  local_param_loader.loadParam("outputs/distance_sensor", (bool &)_capabilities_.produces_distance_sensor);
  local_param_loader.loadParam("outputs/gnss", (bool &)_capabilities_.produces_gnss);
  local_param_loader.loadParam("outputs/gnss_status", (bool &)_capabilities_.produces_gnss_status);
  local_param_loader.loadParam("outputs/rtk", (bool &)_capabilities_.produces_rtk);
  local_param_loader.loadParam("outputs/ground_truth", (bool &)_capabilities_.produces_ground_truth);
  local_param_loader.loadParam("outputs/imu", (bool &)_capabilities_.produces_imu);
  local_param_loader.loadParam("outputs/altitude", (bool &)_capabilities_.produces_altitude);
  local_param_loader.loadParam("outputs/magnetometer_heading", (bool &)_capabilities_.produces_magnetometer_heading);
  local_param_loader.loadParam("outputs/magnetic_field", (bool &)_capabilities_.produces_magnetic_field);
  local_param_loader.loadParam("outputs/rc_channels", (bool &)_capabilities_.produces_rc_channels);
  local_param_loader.loadParam("outputs/battery_state", (bool &)_capabilities_.produces_battery_state);
  local_param_loader.loadParam("outputs/position", (bool &)_capabilities_.produces_position);
  local_param_loader.loadParam("outputs/orientation", (bool &)_capabilities_.produces_orientation);
  local_param_loader.loadParam("outputs/velocity", (bool &)_capabilities_.produces_velocity);
  local_param_loader.loadParam("outputs/angular_velocity", (bool &)_capabilities_.produces_angular_velocity);
  local_param_loader.loadParam("outputs/odometry", (bool &)_capabilities_.produces_odometry);

  local_param_loader.loadParam("position_cmd_map", (bool &)position_cmd_map);

  if (!local_param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
  }

  // | --------------------- service clients -------------------- |
  
  sch_mavros_command_long_ = mrs_lib::ServiceClientHandler<mavros_msgs::srv::CommandLong>(node_, "~/mavros_cmd_out", callback_group_);
  sch_mavros_mode_         = mrs_lib::ServiceClientHandler<mavros_msgs::srv::SetMode>(node_, "~/mavros_set_mode_out", callback_group_);
  
  // | --------------------- service server -------------------- |
  /* if (_simulation_) { */
  /*   sh_ignore_ground_truth_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>( */
  /*       node_, "~/ignore_ground_truth_in", [this](const auto &request, const auto &response) { return this->callbackIgnoreGroundTruth(request, response); }, */
  /*       rclcpp::SystemDefaultsQoS(), cbkgrp_ss_); */
  /* } */

  // | ----------------------- subscribers ---------------------- |
  
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.node_name                           = "MrsHwArduroverApi";
  shopts.no_message_timeout                  = rclcpp::Duration::from_seconds(1.0);
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = callback_group_;
  shopts.qos                                 = qos_profile;

  if (_simulation_) {
    init_subscriber_handler(this, sh_ground_truth_, shopts, "~/ground_truth_in", &MrsUsvArduroverApi::callbackGroundTruth, &MrsUsvArduroverApi::timeoutGeneralTopic,
                            error_type_t::not_receiving_ground_truth, "Not receiving ground truth data");
  }

  /* if (!_simulation_) { */
  /* sh_rtk_ = mrs_lib::SubscriberHandler<mrs_modules_msgs::msg::Bestpos>(shopts, "rtk_in", &MrsUsvArduroverApi::callbackRTK, this); */
  /* } */

  init_subscriber_handler(this, sh_mavros_state_, shopts, "~/mavros_state_in", &MrsUsvArduroverApi::callbackMavrosState, &MrsUsvArduroverApi::timeoutGeneralTopic,
                          error_type_t::not_receiving_mavros_state, "Not receiving Mavros state messages");

  sh_mavros_odometry_local_ =
      mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/mavros_local_position_in", &MrsUsvArduroverApi::callbackOdometryLocal, this);

  sh_mavros_odometry_in_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/mavros_odometry_in", &MrsUsvArduroverApi::callbackOdometryIn, this);
  
  init_subscriber_handler(this, sh_mavros_gps_, shopts, "~/mavros_global_position_in", &MrsUsvArduroverApi::callbackNavsatFix,
      &MrsUsvArduroverApi::timeoutGeneralTopic, error_type_t::not_receiving_gps, "Not receiving GPS data");

  init_subscriber_handler(this, sh_mavros_distance_sensor_, shopts, "~/mavros_garmin_in", &MrsUsvArduroverApi::callbackDistanceSensor,
                          &MrsUsvArduroverApi::timeoutGeneralTopic, error_type_t::not_receiving_distance_sensor, "Not receiving distance sensor data");

  init_subscriber_handler(this, sh_mavros_imu_, shopts, "~/mavros_imu_in", &MrsUsvArduroverApi::callbackImu, &MrsUsvArduroverApi::timeoutGeneralTopic,
                          error_type_t::not_receiving_imu, "Not receiving IMU data");

  sh_mavros_magnetometer_heading_ =
      mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/mavros_magnetometer_in", &MrsUsvArduroverApi::callbackMagnetometer, this);

  sh_mavros_magnetic_field_ =
      mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField>(shopts, "~/mavros_magnetic_field_in", &MrsUsvArduroverApi::callbackMagneticField, this);

  sh_mavros_altitude_ = mrs_lib::SubscriberHandler<mavros_msgs::msg::Altitude>(shopts, "~/mavros_altitude_in", &MrsUsvArduroverApi::callbackAltitude, this);

  sh_gps_status_raw_ = mrs_lib::SubscriberHandler<mavros_msgs::msg::GPSRAW>(shopts, "~/mavros_gps_status_raw_in", &MrsUsvArduroverApi::callbackGpsStatusRaw, this);

  // In simulation ignore the timeout for RC channels and battery state
  if (_simulation_) {
    shopts.no_message_timeout = mrs_lib::no_timeout;
  }
  
  sh_mavros_rc_ = mrs_lib::SubscriberHandler<mavros_msgs::msg::RCIn>(shopts, "~/mavros_rc_in", &MrsUsvArduroverApi::callbackRC, this);

  sh_mavros_battery_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState>(shopts, "~/mavros_battery_in", &MrsUsvArduroverApi::callbackBattery, this);

  // | ----------------------- publishers ----------------------- |

  ph_mavros_attitude_target_  = mrs_lib::PublisherHandler<mavros_msgs::msg::AttitudeTarget>(node_, "~/mavros_attitude_setpoint_out");
  ph_mavros_actuator_control_ = mrs_lib::PublisherHandler<mavros_msgs::msg::ActuatorControl>(node_, "~/mavros_actuator_control_out");

  ph_mavros_position_target_  = mrs_lib::PublisherHandler<mavros_msgs::msg::PositionTarget>(node_, "~/mavros_position_setpoint_out");
  ph_mavros_velocity_target_  = mrs_lib::PublisherHandler<geometry_msgs::msg::Twist>(node_, "~/mavros_velocity_setpoint_out");
      
  pub_left_usv_thrust_cmd_    = mrs_lib::PublisherHandler<std_msgs::msg::Float32>(node_, "~/left_usv_thrust_cmd_out");
  pub_right_usv_thrust_cmd_   = mrs_lib::PublisherHandler<std_msgs::msg::Float32>(node_, "~/right_usv_thrust_cmd_out");
  pub_left_usv_thrust_angle_  = mrs_lib::PublisherHandler<std_msgs::msg::Float32>(node_, "~/left_usv_thrust_angle_out");
  pub_right_usv_thrust_angle_ = mrs_lib::PublisherHandler<std_msgs::msg::Float32>(node_, "~/right_usv_thrust_angle_out");

  // | ----------------------- transformer ---------------------- |
  transformer_ = mrs_lib::Transformer(node_);
  transformer_.retryLookupNewest(true);

  // | ----------------------- finish init ---------------------- |

  {
    std::function<void()> callback_fcn = std::bind(&MrsUsvArduroverApi::timerMain, this);

    mrs_lib::TimerHandlerOptions opts;

    opts.node           = node_;
    opts.autostart      = true;
    opts.callback_group = callback_group_;

    timer_main_ = std::make_shared<TimerType>(opts, rclcpp::Rate(10.0, clock_), callback_fcn);
  }

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

//}

/* destroy() //{ */

void MrsUsvArduroverApi::destroy() {

  timer_main_->stop();
}

//}

/* getStatus() //{ */

mrs_msgs::msg::HwApiStatus MrsUsvArduroverApi::getStatus() {

  mrs_msgs::msg::HwApiStatus status;

  status.stamp = clock_->now();

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

mrs_msgs::msg::HwApiCapabilities MrsUsvArduroverApi::getCapabilities() {

  _capabilities_.stamp = clock_->now();

  return _capabilities_;
}

//}

/* callbackControlActuatorCmd() //{ */

bool MrsUsvArduroverApi::callbackActuatorCmd([[maybe_unused]] const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting actuator cmd");
  
  if (!_capabilities_.accepts_actuator_cmd) {
    RCLCPP_ERROR(node_->get_logger(), "the actuator input is not enabled in the config file");
    return false;
  }
  std_msgs::msg::Float32 right_thurst_cmd, left_thurst_cmd, right_thurst_angle, left_thurst_angle;
  
  left_thurst_cmd.data    = msg->motors[0]; 
  right_thurst_cmd.data   = msg->motors[1];
  left_thurst_angle.data  = msg->motors[2];
  right_thurst_angle.data = msg->motors[3];
    
  pub_left_usv_thrust_cmd_.publish(left_thurst_cmd);
  pub_right_usv_thrust_cmd_.publish(right_thurst_cmd); 
  pub_left_usv_thrust_angle_.publish(left_thurst_angle);
  pub_right_usv_thrust_angle_.publish(right_thurst_angle);

  return true;
}

//}

/* callbackControlGroupCmd() //{ */

bool MrsUsvArduroverApi::callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting control group cmd");

  if (!_capabilities_.accepts_control_group_cmd) {
    RCLCPP_ERROR(node_->get_logger(), "the control group input is not enabled in the config file");
    return false;
  }

  mavros_msgs::msg::ActuatorControl msg_out;

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

bool MrsUsvArduroverApi::callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude rate cmd");

  if (!_capabilities_.accepts_attitude_rate_cmd) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "attitude rate input is not enabled in the config file");
    return false;
  }

  mavros_msgs::msg::AttitudeTarget attitude_target;

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

bool MrsUsvArduroverApi::callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude cmd");

  if (!_capabilities_.accepts_attitude_cmd) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "attitude input is not enabled in the config file");
    return false;
  }

  mavros_msgs::msg::AttitudeTarget attitude_target;

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

bool MrsUsvArduroverApi::callbackAccelerationHdgRateCmd([[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg rate cmd");

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool MrsUsvArduroverApi::callbackAccelerationHdgCmd([[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg cmd");

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool MrsUsvArduroverApi::callbackVelocityHdgRateCmd([[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg rate cmd");

  if (!_capabilities_.accepts_velocity_hdg_rate_cmd) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "the velocity and heading rate input is not enabled in the config file");
    return false;
  }
   
  // fill position target command
  geometry_msgs::msg::Twist velocity_cmd;

  velocity_cmd.linear.x = msg->velocity.x;
  velocity_cmd.linear.y = 0;
  velocity_cmd.linear.z = 0;
  velocity_cmd.angular.x = 0;
  velocity_cmd.angular.y = 0;
  velocity_cmd.angular.z = msg->heading_rate;
 
  ph_mavros_velocity_target_.publish(velocity_cmd);

  return true;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool MrsUsvArduroverApi::callbackVelocityHdgCmd([[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg cmd");

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool MrsUsvArduroverApi::callbackPositionCmd([[maybe_unused]] const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting position cmd");
  
  if (!_capabilities_.accepts_position_cmd) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "the position input is not enabled in the config file");
    return false;
  }
  
  mavros_msgs::msg::PositionTarget position_target;
  std::string output_frame = "map";
  if(position_cmd_map)
  {
    output_frame = "map";
    position_target.coordinate_frame = position_target.FRAME_LOCAL_NED;
  } else{
    output_frame = _uav_name_ + "/fcu";
    position_target.coordinate_frame = position_target.FRAME_BODY_NED;
  }

  std::optional<geometry_msgs::msg::TransformStamped> tf_opt;
  {
    std::scoped_lock lock(transformer_mutex);
    tf_opt = transformer_.getTransform(msg->header.frame_id, output_frame);
  }
  if (!tf_opt.has_value()) 
  { 
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 500, "Could not obtain transform from %s to %s", msg->header.frame_id.c_str(), output_frame.c_str());
    return false;
  } 
 
  // transform    
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pose_transformed;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_to_transform;
  pose_to_transform.header = msg->header;
  pose_to_transform.pose.pose.position = msg->position;
  pose_to_transform.pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, msg->heading);
  {
    std::scoped_lock lock(transformer_mutex);
    pose_transformed = transformer_.transform(pose_to_transform, tf_opt.value());
  }
  if (!pose_transformed)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 500, "Failed to transform pose from %s to %s, returning.", msg->header.frame_id.c_str(), output_frame.c_str());
    return false;
  }
   
  // get target heading
  mrs_lib::AttitudeConverter attitude_converter(pose_transformed.value().pose.pose.orientation); 
  double position_target_yaw = attitude_converter.getHeading();
   
  // fill position target command
  position_target.header.stamp = clock_->now();
 
  position_target.type_mask = 0;
  position_target.type_mask += position_target.IGNORE_VX+position_target.IGNORE_VY+position_target.IGNORE_VZ;
  position_target.type_mask += position_target.IGNORE_AFX+position_target.IGNORE_AFY+position_target.IGNORE_AFZ;
  position_target.type_mask += position_target.IGNORE_YAW_RATE;
  position_target.type_mask += position_target.IGNORE_YAW;

  position_target.position.x = pose_transformed.value().pose.pose.position.x;
  position_target.position.y = pose_transformed.value().pose.pose.position.y;
  position_target.position.z = pose_transformed.value().pose.pose.position.z;
  position_target.yaw = position_target_yaw;
 
  // publish position target command
  ph_mavros_position_target_.publish(position_target);
  
  return true;
}

//}

/* callbackTrackerCmd() //{ */

void MrsUsvArduroverApi::callbackTrackerCmd([[maybe_unused]] const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> MrsUsvArduroverApi::callbackArming([[maybe_unused]] const bool& request) {

  std::stringstream ss;

  auto srv_out = std::make_shared<mavros_msgs::srv::CommandLong::Request>();

  srv_out->broadcast    = false;
  srv_out->command      = 400; // the code for arming
  srv_out->confirmation = true;

  srv_out->param1 = request ? 1 : 0;     // arm or disarm?
  srv_out->param2 = request ? 0 : 21196; // 21196 allows to disarm even in mid-flight
  srv_out->param3 = 0;
  srv_out->param4 = 0;
  srv_out->param5 = 0;
  srv_out->param6 = 0;
  srv_out->param7 = 0;

  /* ss << "calling for " << (request ? "arming" : "disarming") << " resulted in failure: '" << srv_out.response.result << "'"; */
  /* ROS_INFO_STREAM_THROTTLE(1.0, "[" << node_name.c_str() << "]: " << ss.str()); */
  /* return {true, ss.str()}; */
  RCLCPP_INFO(node_->get_logger(), "calling for %s", request ? "arming" : "disarming");
  
  bool success = false;

  auto response = sch_mavros_command_long_.callSync(srv_out);

  if (response) {

    success = response.value()->success;

    if (success) {
      
      ss << "service call for " << (request ? "arming" : "disarming") << " was successful";
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    } else {
      
      ss << "service call for " << (request ? "arming" : "disarming") << " failed";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    }

  } else {
    
    ss << "failed to call Mavros CommandLong service";
    RCLCPP_ERROR(node_->get_logger(), "%s", ss.str().c_str());
  }

  return {success, ss.str()};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> MrsUsvArduroverApi::callbackOffboard(void) {

  auto srv_out = std::make_shared<mavros_msgs::srv::SetMode::Request>();

  srv_out->base_mode   = 0;
  srv_out->custom_mode = "GUIDED";

  bool success = false;

  auto response = sch_mavros_mode_.callSync(srv_out);

  std::stringstream ss;

  if (response) {
    
    if (response.value()->mode_sent != 1) {

      ss << "service call for offboard failed, returned " << response.value()->mode_sent;

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "%s", ss.str().c_str());
    
    } else {

      ss << "switched to offboard mode";
      success = true;
    }

  } else {

    ss << "failed to call Mavros SetMode service";
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "%s", ss.str().c_str());
  }
  
  return {success, ss.str()};
}

//}

// | ------------------- additional methods ------------------- |

/* timeoutMavrosState() //{ */

void MrsUsvArduroverApi::timeoutMavrosState(void) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_mavros_state_.hasMsg()) {
    return;
  }
  
  auto last_mavros_state_time = mrs_lib::get_mutexed(mutex_last_mavros_state_time_, last_mavros_state_time_);

  const auto time = clock_->now() - last_mavros_state_time;

  if (time.seconds() > _mavros_timeout_) {

    {
      std::scoped_lock lock(mutex_status_);

      connected_ = false;
      offboard_  = false;
      armed_     = false;
      mode_      = "";
    }

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Have not received Mavros state for more than '%.3f s'", time.seconds());

  } 

  if (time.seconds() > _mavros_passable_delay_) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Not receiving Mavros state message for '%.3f s'! Setup the PixHawk SD card!!", time.seconds());
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000,
                         "This could be also caused by the not being PixHawk booted properly due to, e.g., antispark connector jerkiness.");
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000,
                         "The Mavros state should be supplied at 100 Hz to provide fast refresh rate on the state of the OFFBOARD mode.");
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000,
                         "If missing, the UAV could be disarmed by safety routines while not knowing it has switched to the MANUAL mode.");
  }
}

//}


/* timeoutGeneralTopic() //{ */

void MrsUsvArduroverApi::timeoutGeneralTopic(const std::string &topic_name, const rclcpp::Time &last_msg_time, const error_type_t &error_type,
                                       const std::string &error_description) {

  if (!is_initialized_) {
    return;
  }

  const auto delay = clock_->now() - last_msg_time;

  if (delay > _general_topic_timeout_) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Have not received '%s' for more than '%.3f s'", topic_name.c_str(), delay.seconds());

    error_publisher_->addGeneralError(error_type, error_description);
  }
}

//}

/* RCChannelToRange() //{ */

double MrsUsvArduroverApi::RCChannelToRange(const double& rc_value) {

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

void MrsUsvArduroverApi::callbackMavrosState(const mavros_msgs::msg::State::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting Mavros state");

  {
    std::scoped_lock lock(mutex_status_);

    offboard_  = msg->mode == "GUIDED";
    armed_     = msg->armed;
    connected_ = true;
    mode_      = msg->mode;
  }

  // | ----------------- publish the diagnostics ---------------- |

  rclcpp::Time timestamp = clock_->now();
  
  mrs_msgs::msg::HwApiStatus status;

  {
    std::scoped_lock lock(mutex_status_);

    status.stamp     = timestamp;
    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  mrs_lib::set_mutexed(mutex_last_mavros_state_time_, timestamp, last_mavros_state_time_);
  
  common_handlers_->publishers.publishStatus(status);
}

//}

/* callbackOdometryLocal() //{ */

void MrsUsvArduroverApi::callbackOdometryLocal(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting Mavros's local odometry");

  nav_msgs::msg::Odometry odom = *msg;
  
  {
    std::scoped_lock lock(mutex_orientation_);
    odom.pose.pose.orientation = orientation_;
  }
  {
    std::scoped_lock lock(mutex_angular_velocity_);
    odom.twist.twist.angular = angular_velocity_;
  }

  // | -------------------- publish position -------------------- |

  if (_capabilities_.produces_position) {

  geometry_msgs::msg::PointStamped position;

    // TODO: Investigate potential inconsistencies between ArduPilot projection and MRS UTM conversion

    position.header.stamp    = odom.header.stamp;
    position.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    position.point           = odom.pose.pose.position;

    common_handlers_->publishers.publishPosition(position);
  }

  // | ------------------- publish orientation ------------------ |

  /* if (_capabilities_.produces_orientation) { */

  /*   geometry_msgs::QuaternionStamped orientation; */

  /*   orientation.header.stamp    = odom->header.stamp; */
  /*   orientation.header.frame_id = _uav_name_ + "/" + _world_frame_name_; */
  /*   orientation.quaternion      = odom->pose.pose.orientation; */

  /*   common_handlers_->publishers.publishOrientation(orientation); */
  /* } */

  // | -------------------- publish velocity -------------------- |

  if (_capabilities_.produces_velocity) {

    geometry_msgs::msg::Vector3Stamped velocity;

    velocity.header.stamp    = odom.header.stamp;
    velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
    velocity.vector          = odom.twist.twist.linear;

    common_handlers_->publishers.publishVelocity(velocity);
  }

  // | ---------------- publish angular velocity ---------------- |

  /* if (_capabilities_.produces_angular_velocity) { */

  /*   geometry_msgs::Vector3Stamped angular_velocity; */

  /*   angular_velocity.header.stamp    = odom->header.stamp; */
  /*   angular_velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_; */
  /*   angular_velocity.vector          = odom->twist.twist.angular; */

  /*   common_handlers_->publishers.publishAngularVelocity(angular_velocity); */
  /* } */

  // | -------------------- publish odometry -------------------- |

  if (_capabilities_.produces_odometry) {
    common_handlers_->publishers.publishOdometry(odom);
  }

}

//}

/* callbackOdometryIn() //{ */

void MrsUsvArduroverApi::callbackOdometryIn(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting Mavros's odometry in");

  auto odom = msg;

  {
    std::scoped_lock lock(mutex_orientation_);
    orientation_ = odom->pose.pose.orientation;
  }
  {
    std::scoped_lock lock(mutex_angular_velocity_);
    angular_velocity_ = odom->twist.twist.angular;
  }

  // | ------------------- publish orientation ------------------ |

  if (_capabilities_.produces_orientation) {

    geometry_msgs::msg::QuaternionStamped orientation;

    orientation.header.stamp    = odom->header.stamp;
    orientation.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    orientation.quaternion      = odom->pose.pose.orientation;

    common_handlers_->publishers.publishOrientation(orientation);
  }

  // | ---------------- publish angular velocity ---------------- |

  if (_capabilities_.produces_angular_velocity) {

    geometry_msgs::msg::Vector3Stamped angular_velocity;

    angular_velocity.header.stamp    = odom->header.stamp;
    angular_velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
    angular_velocity.vector          = odom->twist.twist.angular;

    common_handlers_->publishers.publishAngularVelocity(angular_velocity);
  }
}

//}

/* callbackNavsatFix() //{ */

void MrsUsvArduroverApi::callbackNavsatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting NavSat fix");

  if (_capabilities_.produces_gnss) {

    common_handlers_->publishers.publishGNSS(*msg);
  }
}

//}

/* callbackDistanceSensor() //{ */

void MrsUsvArduroverApi::callbackDistanceSensor(const sensor_msgs::msg::Range::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting distnace sensor");

  if (_capabilities_.produces_distance_sensor) {

    common_handlers_->publishers.publishDistanceSensor(*msg);
  }
}

//}

/* callbackImu() //{ */

void MrsUsvArduroverApi::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting IMU");

  if (_capabilities_.produces_imu) {

    sensor_msgs::msg::Imu new_imu_msg = *msg;
    new_imu_msg.header.frame_id  = _uav_name_ + "/" + _body_frame_name_;

    common_handlers_->publishers.publishIMU(new_imu_msg);
  }
}

//}

/* callbackCompass() //{ */

void MrsUsvArduroverApi::callbackMagnetometer(const std_msgs::msg::Float64::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting magnetometer heading");

  if (_capabilities_.produces_magnetometer_heading) {
    
    // Converting the value from MAVROS msg in degrees into radians for
    // consistency with other values.
    // Mavros yaw angle is given in degrees from 0.0..359.99 degrees
    auto global_heading_rad = mrs_lib::geometry::degrees::convert<mrs_lib::geometry::radians>(msg->data);

    // To be consistent with the local heading [-pi, pi]
    auto sradians_heading = global_heading_rad.convert<mrs_lib::geometry::sradians>();

    mrs_msgs::msg::Float64Stamped mag_out;
    mag_out.header.stamp    = clock_->now();
    mag_out.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    mag_out.value           = sradians_heading.value();

    common_handlers_->publishers.publishMagnetometerHeading(mag_out);
  }
}

//}

/* callbackMagneticField() //{ */

void MrsUsvArduroverApi::callbackMagneticField(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting magnetic field");

  if (_capabilities_.produces_magnetic_field) {

    common_handlers_->publishers.publishMagneticField(*msg);
  }
}

//}

/* callbackRC() //{ */

void MrsUsvArduroverApi::callbackRC(const mavros_msgs::msg::RCIn::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting RC");

  if (_capabilities_.produces_rc_channels) {

    mrs_msgs::msg::HwApiRcChannels rc_out;

    rc_out.stamp = msg->header.stamp;

    for (size_t i = 0; i < msg->channels.size(); i++) {
      rc_out.channels.push_back(RCChannelToRange(msg->channels[i]));
    }

    common_handlers_->publishers.publishRcChannels(rc_out);
  }
}

//}

/* callbackAltitude() //{ */

void MrsUsvArduroverApi::callbackAltitude(const mavros_msgs::msg::Altitude::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting Altitude");

  if (_capabilities_.produces_altitude) {

    mrs_msgs::msg::HwApiAltitude altitude_out;

    altitude_out.stamp = msg->header.stamp;
    altitude_out.amsl  = msg->amsl;

    common_handlers_->publishers.publishAltitude(altitude_out);
  }
}

//}

/* callbackGpsStatusRaw() //{ */

void MrsUsvArduroverApi::callbackGpsStatusRaw(const mavros_msgs::msg::GPSRAW::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting Gps Status Raw");

  if (_capabilities_.produces_gnss_status) {

    mrs_msgs::msg::GpsInfo gps_info_out;

    gps_info_out.stamp    = msg->header.stamp; // [GPS_FIX_TYPE] GPS fix type
    gps_info_out.fix_type = msg->fix_type;     // [GPS_FIX_TYPE] GPS fix type

    gps_info_out.lat                = double(msg->lat) / 10000000; // [deg] Latitude (WGS84, EGM96 ellipsoid)
    gps_info_out.lon                = double(msg->lon) / 10000000; // [deg] Longitude (WGS84, EGM96 ellipsoid)
    gps_info_out.alt                = float(msg->alt) / 1000;      // [m]  (MSL). Positive for up. Not WGS84 altitude.
    gps_info_out.eph                = msg->eph;                    // GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
    gps_info_out.epv                = msg->epv;                    // GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
    gps_info_out.vel                = float(msg->vel) / 100;       // [m/s] GPS ground speed. If unknown, set to: UINT16_MAX
    gps_info_out.cog                = float(msg->cog) / 100;       // [deg] Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees.
    gps_info_out.satellites_visible = msg->satellites_visible;     // Number of satellites visible. If unknown, set to 255

    gps_info_out.alt_ellipsoid = float(msg->alt_ellipsoid) / 1000; // [m] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
    gps_info_out.h_acc         = float(msg->h_acc) / 1000;         // [m] Position uncertainty. Positive for up.
    gps_info_out.v_acc         = float(msg->v_acc) / 1000;         // [m] Altitude uncertainty. Positive for up.
    gps_info_out.vel_acc       = float(msg->vel_acc) / 1000;       // [m/s] Speed uncertainty. Positive for up.
    gps_info_out.hdg_acc       = float(msg->hdg_acc) / 1000;       // [deg] Heading / track uncertainty
    gps_info_out.yaw           = float(msg->yaw) / 100;            // [deg] Yaw in earth frame from north.
    gps_info_out.dgps_num_sats = msg->dgps_numch;                  // Number of DGPS satellites
    gps_info_out.dgps_age      = float(msg->dgps_age) / 1000;      // [s] Age of DGPS info
    gps_info_out.baseline_dist = 0;                                // [m] distance to the basestation, not supported by the GPSRAW message

    common_handlers_->publishers.publishGNSSStatus(gps_info_out);
  }
}

//}

/* callbackBattery() //{ */

void MrsUsvArduroverApi::callbackBattery(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting battery");

  if (_capabilities_.produces_battery_state) {

    common_handlers_->publishers.publishBatteryState(*msg);
  }
}

//}

/* callbackGroundTruth() //{ */

void MrsUsvArduroverApi::callbackGroundTruth(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting ground truth");

  auto odom = msg;

  // | ------------------ publish ground truth ------------------ |

  if (_capabilities_.produces_ground_truth) {

    nav_msgs::msg::Odometry gt = *msg;

    // if frame_id is "/world", "world", "/map" or "map" gazebo reports velocitites in global world frame so we need to transform them to body frame
    if (msg->header.frame_id == "/world" || msg->header.frame_id == "world" || msg->header.frame_id == "/map" || msg->header.frame_id == "map") {

      RCLCPP_INFO_ONCE(node_->get_logger(), "transforming Gazebo ground truth velocities from world to body frame");

      Eigen::Matrix3d R = mrs_lib::AttitudeConverter(msg->pose.pose.orientation);

      Eigen::Vector3d lin_vel_world(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
      Eigen::Vector3d lin_vel_body = R.inverse() * lin_vel_world;

      Eigen::Vector3d angular_vel_world(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
      Eigen::Vector3d angular_vel_body = R.inverse() * angular_vel_world;

      gt.twist.twist.linear.x = lin_vel_body[0];
      gt.twist.twist.linear.y = lin_vel_body[1];
      gt.twist.twist.linear.z = lin_vel_body[2];

      gt.twist.twist.angular.x = angular_vel_body[0];
      gt.twist.twist.angular.y = angular_vel_body[1];
      gt.twist.twist.angular.z = angular_vel_body[2];
    }

    common_handlers_->publishers.publishGroundTruth(gt);
  }

  if (_capabilities_.produces_rtk) {

    double lat;
    double lon;

    mrs_lib::UTMtoLL(msg->pose.pose.position.y + _sim_rtk_utm_y_, msg->pose.pose.position.x + _sim_rtk_utm_x_, _sim_rtk_utm_zone_, lat, lon);

    sensor_msgs::msg::NavSatFix gnss;

    gnss.header.stamp = msg->header.stamp;

    gnss.latitude  = lat;
    gnss.longitude = lon;
    gnss.altitude  = msg->pose.pose.position.z + _sim_rtk_amsl_;

    mrs_msgs::msg::RtkGps rtk;

    rtk.header.stamp    = msg->header.stamp;
    rtk.header.frame_id = "gps";

    rtk.gps.latitude      = lat;
    rtk.gps.longitude     = lon;
    rtk.gps.altitude      = msg->pose.pose.position.z + _sim_rtk_amsl_;
    rtk.gps.covariance[0] = std::pow(0.1, 2);
    rtk.gps.covariance[4] = std::pow(0.1, 2);
    rtk.gps.covariance[8] = std::pow(0.1, 2);

    rtk.fix_type.fix_type = rtk.fix_type.RTK_FIX;

    rtk.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;

    common_handlers_->publishers.publishRTK(rtk);
  }
}

//}

// callbackIgnoreGroundTruth() //{ */

bool MrsUsvArduroverApi::callbackIgnoreGroundTruth(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr resp) {
  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.node_name                           = "MrsUsvArduroverApi";
  shopts.no_message_timeout                  = rclcpp::Duration::from_seconds(1.0);
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = callback_group_;

  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  shopts.qos = qos_profile;


  if (req->data) {
    resp->message = "Stopping subscriber of ground_truth.";
    if (sh_ground_truth_.hasMsg()) {
      sh_ground_truth_.stop();
    }
  } else {
    resp->message = "Starting subscriber of ground_truth.";
    init_subscriber_handler(this, sh_ground_truth_, shopts, "~/ground_truth_in", &MrsUsvArduroverApi::callbackGroundTruth, &MrsUsvArduroverApi::timeoutGeneralTopic,
                            error_type_t::not_receiving_ground_truth, "Not receiving ground truth.");
  }
  resp->success = true;
  return true;
}

//}

/* /1* callbackRTK() //{ *1/ */

/* void MrsUsvArduroverApi::callbackRTK(const mrs_modules_msgs::Bestpos::ConstPtr msg) { */

/*   if (!is_initialized_) { */
/*     return; */
/*   } */

/*   ROS_INFO_ONCE("[MrsUsvArduroverApi]: getting rtk"); */

/*   mrs_msgs::RtkGps rtk_msg_out; */

/*   rtk_msg_out.gps.latitude  = msg->latitude; */
/*   rtk_msg_out.gps.longitude = msg->longitude; */
/*   rtk_msg_out.gps.altitude  = msg->height; */

/*   rtk_msg_out.header.stamp    = ros::Time::now(); */
/*   rtk_msg_out.header.frame_id = _uav_name_ + "/" + _body_frame_name_; */

/*   if (msg->position_type == "L1_INT") { */
/*     rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; */
/*     rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.RTK_FIX; */

/*   } else if (msg->position_type == "L1_FLOAT") { */
/*     rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; */
/*     rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.RTK_FLOAT; */

/*   } else if (msg->position_type == "PSRDIFF") { */
/*     rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; */
/*     rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.DGPS; */

/*   } else if (msg->position_type == "SINGLE") { */
/*     rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; */
/*     rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.SPS; */

/*   } else if (msg->position_type == "NONE") { */
/*     rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_NO_FIX; */
/*     rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.NO_FIX; */
/*   } */

/*   common_handlers_->publishers.publishRTK(rtk_msg_out); */
/* } */

/* //} */

/* timerMain() //{ */

void MrsUsvArduroverApi::timerMain() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "main timer spinning");

  timeoutMavrosState();
}

//}

}  // namespace mrs_usv_ardurover_api

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_usv_ardurover_api::MrsUsvArduroverApi, mrs_uav_hw_api::MrsUavHwApi)
