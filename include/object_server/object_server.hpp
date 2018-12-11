#ifndef __OBJECT_SERVER__
#define __OBJECT_SERVER__

#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>

/**
  Sets up fixed transforms for coordinated experiments using interactive markers
  and publishes tf transforms accordingly.

  Markers are loaded from the parameter server during object construction, or
dynamically through a service call.
**/
class ObjectServer
{
 public:
  ObjectServer();

  /**
    Processes main execution loop where transforms are
    published.
  **/
  void runServer();

 private:
  std::map<std::string,
           std::shared_ptr<interactive_markers::InteractiveMarkerServer> >
      marker_servers_;
  std::map<std::string, std::string> parent_frames_;
  std::map<std::string, geometry_msgs::Pose> object_poses_;
  tf::TransformBroadcaster broadcaster_;
  bool do_update_;
  ros::NodeHandle nh_;
  ros::ServiceServer update_server_;

  /**
    Loads the initial markers from pre-defined ROS parameters and other server
  parameters.
  **/
  bool init();

  /**
    Sets-up an interactive marker for 6-DOF manipulation.

    @param marker The interactive marker to be configured
  **/
  void setupMarker(visualization_msgs::InteractiveMarker &marker) const;

  /**
    Acquire feedback from the interactive markers
  **/
  void markerFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
    Toggles between updating and not updating the object frames based on the
  interactive markers
  **/
  bool toggleMarkerUpdate(std_srvs::SetBool::Request &req,
                          std_srvs::SetBool::Response &res);
};
#endif
