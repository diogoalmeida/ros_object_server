#include <object_server/object_server.hpp>

ObjectServer::ObjectServer() : do_update_(true), nh_("~")
{
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("obj_server");
  obj1_name_ = "obj1_marker";
  obj2_name_ = "obj2_marker";

  obj1_.scale = 0.1;
  obj1_.header.frame_id = "left_gripper";
  obj1_.header.stamp=ros::Time::now();
  obj1_.name = obj1_name_;
  obj1_.description = "Marker for obj1";

  obj2_ = obj1_;
  obj2_.header.frame_id = "right_gripper";
  obj2_.name = obj2_name_;
  obj2_.description = "Marker for obj2";

  obj1_pose_.orientation.x = 0.0;
  obj1_pose_.orientation.y = 0.0;
  obj1_pose_.orientation.z = 0.0;
  obj1_pose_.orientation.w = 1.0;
  obj2_pose_.orientation.x = 0.707;
  obj2_pose_.orientation.y = 0.0;
  obj2_pose_.orientation.z = 0.0;
  obj2_pose_.orientation.w = 0.707;

  setupMarker(obj1_);
  setupMarker(obj2_);

  update_server_ = nh_.advertiseService("obj_marker_update", &ObjectServer::toggleMarkerUpdate, this);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server_->insert(obj1_, boost::bind(&ObjectServer::markerFeedback, this, _1));
  server_->insert(obj2_, boost::bind(&ObjectServer::markerFeedback, this, _1));

  // 'commit' changes and send to all clients
  server_->applyChanges();
}

bool ObjectServer::toggleMarkerUpdate(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  do_update_ = req.data;
  res.success = true;

  if (do_update_)
  {
    res.message = "Updating marker";
  }
  else
  {
    res.message = "Stopping marker update";
  }

  return true;
}

void ObjectServer::runServer()
{
  tf::Transform obj1_transform, obj2_transform;
  while(ros::ok())
  {
    obj1_transform.setOrigin(tf::Vector3(obj1_pose_.position.x, obj1_pose_.position.y, obj1_pose_.position.z));
    obj2_transform.setOrigin(tf::Vector3(obj2_pose_.position.x, obj2_pose_.position.y, obj2_pose_.position.z));
    obj1_transform.setRotation(tf::Quaternion(obj1_pose_.orientation.x, obj1_pose_.orientation.y, obj1_pose_.orientation.z, obj1_pose_.orientation.w));
    obj2_transform.setRotation(tf::Quaternion(obj2_pose_.orientation.x, obj2_pose_.orientation.y, obj2_pose_.orientation.z, obj2_pose_.orientation.w));
    broadcaster_.sendTransform(tf::StampedTransform(obj1_transform, ros::Time::now(), "left_gripper", "obj1"));
    broadcaster_.sendTransform(tf::StampedTransform(obj2_transform, ros::Time::now(), "right_gripper", "obj2"));
    ros::Duration(0.02).sleep();
    ros::spinOnce();
  }
}

void ObjectServer::markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (!do_update_)
  {
    return;
  }

  if (feedback->marker_name == obj1_name_)
  {
    obj1_pose_ = feedback->pose;
  }

  if (feedback->marker_name == obj2_name_)
  {
    obj2_pose_ = feedback->pose;
  }
}


void ObjectServer::setupMarker(visualization_msgs::InteractiveMarker &marker) const
{
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.01;
  box_marker.scale.y = 0.01;
  box_marker.scale.z = 0.01;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  marker.controls.push_back( box_control );

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_server");
  ObjectServer server;
  server.runServer();
}
