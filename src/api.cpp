/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>

//}

namespace mrs_uav_pixhawk_api
{

/* class ControllerModule //{ */

class HwApi : public mrs_uav_managers::Controller {

public:
  ~HwApi(){};

  void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space, const double uav_mass,
                  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &control_reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

private:
  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  double _uav_mass_;

  double hover_thrust_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                             mutex_drs_;
  typedef controller_module::controller_paramsConfig DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t>   Drs_t;
  boost::shared_ptr<Drs_t>                           drs_;
  void                                               callbackDrs(controller_module::controller_paramsConfig &params, uint32_t level);
  DrsParams_t                                        params_;
  std::mutex                                         mutex_params_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void HwApi::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string name, const std::string name_space,
                                  const double uav_mass, std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;

  // | ------------------- loading parameters ------------------- |

  /* mrs_lib::ParamLoader param_loader(nh_, "ControllerModule"); */

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ControllerModule]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[ControllerModule]: initialized");

  is_initialized_ = true;
}

//}

}  // namespace controller_module

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_pixhawk_api::HwApi, mrs_uav_hw_api::MrsUavHwApi)
