/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_hw_api/api.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>

//}

namespace mrs_uav_pixhawk_api
{

/* class Api //{ */

class Api : public mrs_uav_hw_api::MrsUavHwApi {

public:
  ~Api(){};

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
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void Api::initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers,
                     [[maybe_unused]] const std::string &topic_prefix, [[maybe_unused]] const std::string &uav_name) {

  ros::NodeHandle nh_(parent_nh);

  common_handlers_ = common_handlers;

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MrsUavHwApi");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsUavHwDummyApi]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MrsUavHwDummyApi]: initialized");

  is_initialized_ = true;
}

//}

/* getDiagnostics() //{ */

mrs_msgs::HwApiDiagnostics Api::getDiagnostics() {

  mrs_msgs::HwApiDiagnostics diag;

  diag.stamp = ros::Time::now();

  diag.armed    = false;
  diag.offboard = false;
  diag.running  = false;

  return diag;
}

//}

/* getMode() //{ */

mrs_msgs::HwApiMode Api::getMode() {

  mrs_msgs::HwApiMode mode;

  mode.api_name = "Api";
  mode.stamp    = ros::Time::now();

  return mode;
}

//}

/* callbackControlGroupCmd() //{ */

bool Api::callbackControlGroupCmd([[maybe_unused]] const mrs_msgs::HwApiControlGroupCmd &msg) {

  return false;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool Api::callbackAttitudeRateCmd([[maybe_unused]] const mrs_msgs::HwApiAttitudeRateCmd &msg) {

  return false;
}

//}

/* callbackAttitudeCmd() //{ */

bool Api::callbackAttitudeCmd([[maybe_unused]] const mrs_msgs::HwApiAttitudeCmd &msg) {

  return false;
}

//}

/* callbackTranslationCmd() //{ */

bool Api::callbackTranslationCmd([[maybe_unused]] const mrs_msgs::HwApiTranslationCmd &msg) {

  return false;
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> Api::callbackArming([[maybe_unused]] const bool &request) {

  return {false, "Dummy interface does not allow to arm."};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> Api::callbackOffboard([[maybe_unused]] const bool &request) {

  return {false, "Dummy interface does not allow to switch to offboard."};
}

//}

}  // namespace mrs_uav_pixhawk_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_pixhawk_api::Api, mrs_uav_hw_api::MrsUavHwApi)
