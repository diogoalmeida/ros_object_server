#ifndef __OBJECT_SERVER__
#define __OBJECT_SERVER__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/SetBool.h>
#include <interactive_markers/interactive_marker_server.h>

/**
  Sets up fixed transforms for coordinated experiments using interactive markers
  and publishes tf transforms accordingly.
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
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  tf::TransformBroadcaster broadcaster_;
  visualization_msgs::InteractiveMarker obj1_, obj2_;
  geometry_msgs::Pose obj1_pose_, obj2_pose_;
  std::string obj1_name_, obj2_name_;
  bool do_update_;
  ros::NodeHandle nh_;
  ros::ServiceServer update_server_;

  /**
    Sets-up an interactive marker for 6-DOF manipulation.

    @param marker The interactive marker to be configured
  **/
  void setupMarker(visualization_msgs::InteractiveMarker &marker) const;

  /**
    Acquire feedback from the interactive markers
  **/
  void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
    Toggles between updating and not updating the object frames based on the interactive markers
  **/
  bool toggleMarkerUpdate(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

};
#endif
