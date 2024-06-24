#define VERSION "1.0.4.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>

#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ValidateReference.h>
#include <mrs_msgs/SpawnerDiagnostics.h>

#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/CameraInfo.h>

#include <topic_tools/shape_shifter.h>

//}

namespace mrs_uav_general
{

namespace automatic_start
{

/* class Topic //{ */

class Topic {
private:
  std::string topic_name_;
  ros::Time   last_time_;

public:
  Topic(std::string topic_name) : topic_name_(topic_name) {
    last_time_ = ros::Time(0);
  }

  void updateTime(void) {
    last_time_ = ros::Time::now();
  }

  ros::Time getTime(void) {
    return last_time_;
  }

  std::string getTopicName(void) {
    return topic_name_;
  }
};

//}

/* class AutomaticStart //{ */

// state machine
typedef enum
{

  STATE_IDLE,
  STATE_TAKEOFF,
  STATE_IN_ACTION,
  STATE_LAND,
  STATE_FINISHED

} LandingStates_t;

const char* state_names[5] = {

    "IDLING", "TAKING OFF", "IN ACTION", "LANDING", "FINISHED"};

class AutomaticStart : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  std::string _uav_name_;
  bool        _simulation_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient service_client_motors_;
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_land_home_;
  ros::ServiceClient service_client_land_;
  ros::ServiceClient service_client_eland_;
  ros::ServiceClient service_client_validate_reference_;
  ros::ServiceClient service_client_start_;
  ros::ServiceClient service_client_stop_;

  // | ----------------------- subscribers ---------------------- |

  ros::Subscriber subscriber_mavros_state_;
  ros::Subscriber subscriber_control_manager_diagnostics_;
  ros::Subscriber subscriber_spawner_diagnostics_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher publisher_can_takeoff_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);
  double     _main_timer_rate_;

  // | ---------------------- mavros state ---------------------- |

  void       callbackMavrosState(const mavros_msgs::StateConstPtr& msg);
  bool       got_mavros_state_ = false;
  std::mutex mutex_mavros_state_;

  // | ---------------------- mavros state ---------------------- |

  void                         callbackSpawnerDiagnostics(const mrs_msgs::SpawnerDiagnosticsConstPtr& msg);
  bool                         got_spawner_diagnostics = false;
  mrs_msgs::SpawnerDiagnostics spawner_diagnostics_;
  std::mutex                   mutex_spawner_diagnostics_;

  // | ----------------- arm and offboard check ----------------- |

  ros::Time armed_time_;
  bool      armed_ = false;

  ros::Time offboard_time_;
  bool      offboard_ = false;

  // | --------------- control manager diagnostics -------------- |

  void                                callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  std::mutex                          mutex_control_manager_diagnostics_;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_;
  bool                                got_control_manager_diagnostics_ = false;

  // | ------------------------ routines ------------------------ |

  bool takeoff();

  bool landImpl();
  bool elandImpl();
  bool landHomeImpl();
  bool land();
  bool validateReference();

  bool setMotors(const bool value);
  bool disarm();
  bool start(void);
  bool stop();

  // | ---------------------- other params ---------------------- |

  double      _action_duration_;
  double      _pre_takeoff_sleep_;
  bool        _handle_landing_ = false;
  bool        _handle_takeoff_ = false;
  std::string _land_mode_;
  double      _safety_timeout_;

  // | ---------------------- start service --------------------- |

  int       _start_n_attempts_;
  int       call_attempt_counter_ = 0;
  ros::Time start_time_;

  // | ---------------------- state machine --------------------- |

  uint current_state = STATE_IDLE;
  void changeState(LandingStates_t new_state);

  // | ---------------- generic topic subscribers --------------- |

  bool                     _topic_check_enabled_ = false;
  double                   _topic_check_timeout_;
  std::vector<std::string> _topic_check_topic_names_;

  std::vector<Topic>           topic_check_topics_;
  std::vector<ros::Subscriber> generic_subscriber_vec_;

  // generic callback, for any topic, to monitor its rate
  void genericCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name, const int id);
};

//}

/* onInit() //{ */

void AutomaticStart::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  armed_      = false;
  armed_time_ = ros::Time(0);

  offboard_      = false;
  offboard_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "AutomaticStart");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[AutomaticStart]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("simulation", _simulation_);

  param_loader.loadParam("safety_timeout", _safety_timeout_);
  param_loader.loadParam("main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("call_n_attempts", _start_n_attempts_);

  param_loader.loadParam("land_mode", _land_mode_);
  param_loader.loadParam("handle_landing", _handle_landing_);
  param_loader.loadParam("handle_takeoff", _handle_takeoff_);
  param_loader.loadParam("action_duration", _action_duration_);
  param_loader.loadParam("pre_takeoff_sleep", _pre_takeoff_sleep_);

  param_loader.loadParam("topic_check/enabled", _topic_check_enabled_);
  param_loader.loadParam("topic_check/timeout", _topic_check_timeout_);
  param_loader.loadParam("topic_check/topics", _topic_check_topic_names_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[AutomaticStart]: Could not load all parameters!");
    ros::shutdown();
  }

  // recaltulate the acion duration to seconds
  _action_duration_ *= 60;

  if (!(_land_mode_ == "land_home" || _land_mode_ == "land" || _land_mode_ == "eland")) {

    ROS_ERROR("[AutomaticStart]: land_mode ('%s') was specified wrongly, will eland by default!!!", _land_mode_.c_str());
  }

  // | ----------------------- subscribers ---------------------- |

  subscriber_mavros_state_ = nh_.subscribe("mavros_state_in", 1, &AutomaticStart::callbackMavrosState, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_ =
      nh_.subscribe("control_manager_diagnostics_in", 1, &AutomaticStart::callbackControlManagerDiagnostics, this, ros::TransportHints().tcpNoDelay());
  subscriber_spawner_diagnostics_ =
      nh_.subscribe("spawner_diagnostics_in", 1, &AutomaticStart::callbackSpawnerDiagnostics, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_can_takeoff_ = nh_.advertise<std_msgs::Bool>("can_takeoff_out", 1);

  // | --------------------- service clients -------------------- |

  service_client_takeoff_   = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_land_home_ = nh_.serviceClient<std_srvs::Trigger>("land_home_out");
  service_client_land_      = nh_.serviceClient<std_srvs::Trigger>("land_out");
  service_client_eland_     = nh_.serviceClient<std_srvs::Trigger>("eland_out");
  service_client_motors_    = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  service_client_arm_       = nh_.serviceClient<mavros_msgs::CommandBool>("arm_out");

  service_client_validate_reference_ = nh_.serviceClient<mrs_msgs::ValidateReference>("validate_reference_out");

  service_client_start_ = nh_.serviceClient<std_srvs::Trigger>("start_out");

  service_client_stop_ = nh_.serviceClient<std_srvs::Trigger>("stop_out");

  // | ------------------ setup generic topics ------------------ |

  if (_topic_check_enabled_) {

    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;

    for (int i = 0; i < int(_topic_check_topic_names_.size()); i++) {

      std::string topic_name = _topic_check_topic_names_[i];

      if (topic_name.at(0) != '/') {
        topic_name = "/" + _uav_name_ + "/" + topic_name;
      }

      Topic tmp_topic(topic_name);
      topic_check_topics_.push_back(tmp_topic);

      int id = i;  // id to identify which topic called the generic callback

      callback                       = [this, topic_name, id](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { genericCallback(msg, topic_name, id); };
      ros::Subscriber tmp_subscriber = nh_.subscribe(topic_name, 1, callback);

      generic_subscriber_vec_.push_back(tmp_subscriber);
    }
  }

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &AutomaticStart::timerMain, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* genericCallback() //{ */

void AutomaticStart::genericCallback([[maybe_unused]] const topic_tools::ShapeShifter::ConstPtr& msg, [[maybe_unused]] const std::string& topic_name,
                                     const int id) {
  topic_check_topics_[id].updateTime();
}

//}

/* callbackMavrosState() //{ */

void AutomaticStart::callbackMavrosState(const mavros_msgs::StateConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStart]: getting mavros state");

  std::scoped_lock lock(mutex_mavros_state_);

  // check armed_ state
  if (armed_ == false) {

    // if armed_ state changed to true, please "start the clock"
    if (msg->armed > 0) {

      armed_      = true;
      armed_time_ = ros::Time::now();
    }

    // if we were armed_ previously
  } else if (armed_ == true) {

    // and we are not really now
    if (msg->armed == 0) {

      armed_ = false;
    }
  }

  // check offboard_ state
  if (offboard_ == false) {

    // if offboard_ state changed to true, please "start the clock"
    if (msg->mode == "OFFBOARD") {

      offboard_      = true;
      offboard_time_ = ros::Time::now();
    }

    // if we were in offboard_ previously
  } else if (offboard_ == true) {

    // and we are not really now
    if (msg->mode != "OFFBOARD") {

      offboard_ = false;
    }
  }

  got_mavros_state_ = true;
}

//}

/* callbackControlManagerDiagnostics() //{ */

void AutomaticStart::callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStart]: getting control manager diagnostics");

  {
    std::scoped_lock lock(mutex_control_manager_diagnostics_);

    control_manager_diagnostics_ = *msg;

    got_control_manager_diagnostics_ = true;
  }
}

//}

/* callbackSpawnerDiagnostics() //{ */

void AutomaticStart::callbackSpawnerDiagnostics(const mrs_msgs::SpawnerDiagnosticsConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStart]: getting spawner diagnostics");

  {
    std::scoped_lock lock(mutex_spawner_diagnostics_);

    spawner_diagnostics_ = *msg;

    got_spawner_diagnostics = true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void AutomaticStart::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!got_control_manager_diagnostics_ || !got_mavros_state_) {
    ROS_WARN_THROTTLE(5.0, "[AutomaticStart]: waiting for data: ControManager=%s, Mavros=%s", got_control_manager_diagnostics_ ? "true" : "FALSE",
                      got_mavros_state_ ? "true" : "FALSE");
    return;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);

  bool   motors           = control_manager_diagnostics.motors;
  double time_from_arming = (ros::Time::now() - armed_time).toSec();
  bool   position_valid   = validateReference();

  bool got_topics = true;

  std::stringstream missing_topics;

  if (_topic_check_enabled_) {

    for (int i = 0; i < int(topic_check_topics_.size()); i++) {
      if ((ros::Time::now() - topic_check_topics_[i].getTime()).toSec() > _topic_check_timeout_) {
        missing_topics << std::endl << "\t" << topic_check_topics_[i].getTopicName();
        got_topics = false;
      }
    }
  }

  std_msgs::Bool can_takeoff_msg;
  can_takeoff_msg.data = false;

  if (got_topics && position_valid && current_state == STATE_IDLE) {
    can_takeoff_msg.data = true;
  }

  try {
    publisher_can_takeoff_.publish(can_takeoff_msg);
  }
  catch (...) {
    ROS_ERROR("exception caught, could not publish");
  }

  if (!got_topics) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[AutomaticStart]: missing data on topics: " << missing_topics.str());
  }

  switch (current_state) {

    case STATE_IDLE: {

      if (armed && !motors) {

        if (position_valid && got_topics) {

          bool res = setMotors(true);

          if (!res) {
            ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: could not set motors ON");
          }
        }

        if (time_from_arming > 1.5) {

          ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: could not set motors ON for 1.5 secs, disarming");
          disarm();
        }
      }

      if (_simulation_) {

        std::scoped_lock lock(mutex_spawner_diagnostics_);

        if (got_spawner_diagnostics) {

          if (!spawner_diagnostics_.spawn_called || spawner_diagnostics_.processing) {
            ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: (simulation) waiting for spawner to finish spawning UAVs");
            return;
          }

        } else {

          ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: (simulation) missing spawner diagnostics");
          return;
        }
      }

      // when armed and in offboard, takeoff
      if (armed && offboard && motors) {

        double armed_time_diff    = (ros::Time::now() - armed_time).toSec();
        double offboard_time_diff = (ros::Time::now() - offboard_time).toSec();

        if ((armed_time_diff > _safety_timeout_) && (offboard_time_diff > _safety_timeout_)) {

          if (_handle_takeoff_) {
            changeState(STATE_TAKEOFF);
          } else {
            changeState(STATE_IN_ACTION);
          }

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff : offboard_time_diff;

          ROS_WARN_THROTTLE(1.0, "starting in %.0f", (_safety_timeout_ - min));
        }
      }

      break;
    }

    case STATE_TAKEOFF: {

      std::scoped_lock lock(mutex_control_manager_diagnostics_);

      // if takeoff finished
      if (control_manager_diagnostics.active_tracker != "NullTracker" && control_manager_diagnostics.active_tracker != "LandoffTracker" &&
          !control_manager_diagnostics.tracker_status.have_goal) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: takeoff finished");

        changeState(STATE_IN_ACTION);
      }

      break;
    }

    case STATE_IN_ACTION: {

      if (_handle_landing_) {

        double in_action_time = (ros::Time::now() - start_time_).toSec();

        ROS_INFO_THROTTLE(5.0, "[AutomaticStart]: in action for %.0f second out of %.0f", in_action_time, _action_duration_);

        if (in_action_time > _action_duration_) {

          ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: the action duration time has passed, landing");

          changeState(STATE_LAND);
        }

      } else {

        changeState(STATE_FINISHED);
      }

      break;
    }

    case STATE_LAND: {

      if (!armed || !offboard || !motors) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: the UAV has probably landed");

        changeState(STATE_FINISHED);
      }

      break;
    }

    case STATE_FINISHED: {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: we are done here");

      break;
    }
  }

}  // namespace automatic_start

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

void AutomaticStart::changeState(LandingStates_t new_state) {

  ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: switching states %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

    case STATE_IDLE: {

      break;
    }

    case STATE_TAKEOFF: {

      if (_pre_takeoff_sleep_ > 1.0) {
        ROS_INFO("[AutomaticStart]: sleeping for %.2f secs before takeoff", _pre_takeoff_sleep_);
        ros::Duration(_pre_takeoff_sleep_).sleep();
      }

      bool res = takeoff();

      if (!res) {
        return;
      }

      break;
    }

    case STATE_IN_ACTION: {

      bool res = start();

      if (!res) {

        if (++call_attempt_counter_ < _start_n_attempts_) {

          ROS_WARN("[AutomaticStart]: failed to call start, attempting again");
          return;

        } else {

          ROS_ERROR("[AutomaticStart]: failed to call start for the %dth time, giving up", call_attempt_counter_);
        }
      }

      call_attempt_counter_ = 0;

      start_time_ = ros::Time::now();

      break;
    }

    case STATE_LAND: {

      {
        bool res = stop();

        if (++call_attempt_counter_ < _start_n_attempts_) {

          ROS_WARN("[AutomaticStart]: failed to call stop, attempting again");

          if (!res) {
            return;
          }

        } else {

          ROS_ERROR("[AutomaticStart]: failed to call stop for the %dth time, giving up", call_attempt_counter_);
        }
      }

      call_attempt_counter_ = 0;

      {
        bool res = land();

        if (!res) {
          return;
        }
      }

      break;
    }

    case STATE_FINISHED: {

      break;
    }

    break;
  }

  current_state = new_state;
}

//}

/* takeoff() //{ */

bool AutomaticStart::takeoff() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: taking off");

  std_srvs::Trigger srv;

  bool res = service_client_takeoff_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: taking off failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for taking off failed");
  }

  return false;
}

//}

/* landHomeImpl() //{ */

bool AutomaticStart::landHomeImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: landing home");

  std_srvs::Trigger srv;

  bool res = service_client_land_home_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: landing home failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for landing home failed");
  }

  return false;
}

//}

/* landImpl() //{ */

bool AutomaticStart::landImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: landing");

  std_srvs::Trigger srv;

  bool res = service_client_land_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: landing failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for landing failed");
  }

  return false;
}

//}

/* elandImpl() //{ */

bool AutomaticStart::elandImpl() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: elanding");

  std_srvs::Trigger srv;

  bool res = service_client_eland_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: elanding failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for elanding failed");
  }

  return false;
}

//}

/* land() //{ */

bool AutomaticStart::land() {

  bool res;

  if (_land_mode_ == "land") {

    res = landImpl();

  } else if (_land_mode_ == "land_home") {

    res = landHomeImpl();

  } else {

    res = elandImpl();
  }

  return res;
}

//}

/* validateReference() //{ */

bool AutomaticStart::validateReference() {

  mrs_msgs::ValidateReference srv_out;

  srv_out.request.reference.header.frame_id = "fcu";

  bool res = service_client_validate_reference_.call(srv_out);

  if (res) {

    if (srv_out.response.success) {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: current position is valid");
      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: current position is not valid (safety area, bumper)!");
      return false;
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: current position could not be validated");
    return false;
  }
}

//}

/* motors() //{ */

bool AutomaticStart::setMotors(const bool value) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: setting motors %s", value ? "ON" : "OFF");

  std_srvs::SetBool srv;
  srv.request.data = value;

  bool res = service_client_motors_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: setting motors failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for setting motors failed");
  }

  return false;
}

//}

/* disarm() //{ */

bool AutomaticStart::disarm() {

  if (!got_mavros_state_) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: cannot disarm, missing mavros state!");

    return false;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_mavros_state_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);

  if (offboard) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: cannot disarm, already in offboard mode!");

    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: disarming");

  mavros_msgs::CommandBool srv;
  srv.request.value = 0;

  bool res = service_client_arm_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: disarming failed");
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for disarming failed");
  }

  return false;
}

//}

/* start() //{ */

bool AutomaticStart::start(void) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: starting action");

  std_srvs::Trigger srv;

  bool res = service_client_start_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: starting action failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for starting action failed");
  }

  return false;
}

//}

/* stop() //{ */

bool AutomaticStart::stop() {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: stopping action");

  std_srvs::Trigger srv;

  bool res = service_client_stop_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: stopping action failed failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for stopping action failed");
  }

  return false;
}

//}

}  // namespace automatic_start

}  // namespace mrs_uav_general

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_general::automatic_start::AutomaticStart, nodelet::Nodelet)
