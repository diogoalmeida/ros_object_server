#include <object_server/object_server.hpp>

ObjectServer::ObjectServer() : do_update_(true), nh_("~")
{
  if (!init())
  {
    throw std::logic_error("Failed to initialize the object server!");
  }

  toggle_update_server_ = nh_.advertiseService(
      "toggle_marker_update", &ObjectServer::toggleMarkerUpdate, this);
  set_pose_server_ = nh_.advertiseService(
      "set_marker_pose", &ObjectServer::markerPoseService, this);
}

bool ObjectServer::init()
{
  std::vector<std::string> server_names;

  if (!nh_.getParam("marker_servers", server_names))
  {
    ROS_WARN(
        "No marker_servers parameter available! No marker will be available by "
        "default.");
  }
  else
  {
    for (unsigned int i = 0; i < server_names.size();
         i++)  // cycle through declared marker_servers and initialize the
               // markers for each server
    {
      std::shared_ptr<interactive_markers::InteractiveMarkerServer> new_server =
          std::make_shared<interactive_markers::InteractiveMarkerServer>(
              server_names[i]);
      std::vector<std::string> marker_names;
      if (!nh_.getParam(server_names[i] + "/marker_names", marker_names))
      {
        ROS_ERROR_STREAM("No marker_names parameter configured for "
                         << server_names[i] << "(" << server_names[i]
                         << "/marker_names)");
        return false;
      }

      for (unsigned int j = 0; j < marker_names.size(); j++)
      {
        visualization_msgs::InteractiveMarker new_marker;
        std::string ns = server_names[i] + "/" + marker_names[j];

        if (!nh_.getParam(ns + "/parent_frame", new_marker.header.frame_id))
        {
          ROS_ERROR_STREAM("Missing " << ns << "/parent_frame parameter!");
          return false;
        }

        if (!nh_.getParam(ns + "/name", new_marker.name))
        {
          ROS_ERROR_STREAM("Missing " << ns << "/name parameter!");
          return false;
        }

        std::vector<double> init_pose;
        if (!nh_.getParam(ns + "/initial_pose", init_pose))
        {
          ROS_ERROR_STREAM("Missing " << ns << "/initial_pose parameter!");
          return false;
        }

        if (init_pose.size() != 7)
        {
          ROS_ERROR_STREAM(ns << "/initial_pose parameter must have dimension "
                                 "7 (position + quaternion)");
          return false;
        }

        new_marker.pose.position.x = init_pose[0];
        new_marker.pose.position.y = init_pose[1];
        new_marker.pose.position.z = init_pose[2];
        new_marker.pose.orientation.x = init_pose[3];
        new_marker.pose.orientation.y = init_pose[4];
        new_marker.pose.orientation.z = init_pose[5];
        new_marker.pose.orientation.w = init_pose[6];
        new_marker.scale = 0.1;
        setupMarker(new_marker);

        parent_frames_[new_marker.name] = new_marker.header.frame_id;
        object_poses_[new_marker.name] = new_marker.pose;
        new_server->insert(
            new_marker, boost::bind(&ObjectServer::markerFeedback, this, _1));
        marker_to_server_[new_marker.name] = server_names[i];
      }

      new_server->applyChanges();
      marker_servers_[server_names[i]] = new_server;
    }
  }

  return true;
}

bool ObjectServer::toggleMarkerUpdate(std_srvs::SetBool::Request &req,
                                      std_srvs::SetBool::Response &res)
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

bool ObjectServer::markerPoseService(object_server::SetMarkers::Request &req,
                                     object_server::SetMarkers::Response &res)
{
  std::string server_name;
  for (unsigned int i = 0; i < req.marker_name.size(); i++)
  {
    if (object_poses_.find(req.marker_name[i]) != object_poses_.end())
    {
      object_poses_[req.marker_name[i]] = req.marker_pose[i];
      server_name = marker_to_server_[req.marker_name[i]];
      marker_servers_[server_name]->setPose(req.marker_name[i],
                                            req.marker_pose[i]);
      marker_servers_[server_name]->applyChanges();
      res.success.push_back(true);
    }
    else
    {
      ROS_WARN_STREAM("Got marker pose request for unknown marker "
                      << req.marker_name[i]);
      res.success.push_back(false);
    }
  }

  return true;
}

void ObjectServer::runServer()
{
  tf::Transform tf_transform;
  while (ros::ok())
  {
    for (auto &x : object_poses_)
    {
      tf_transform.setOrigin(tf::Vector3(
          x.second.position.x, x.second.position.y, x.second.position.z));
      tf_transform.setRotation(
          tf::Quaternion(x.second.orientation.x, x.second.orientation.y,
                         x.second.orientation.z, x.second.orientation.w));
      broadcaster_.sendTransform(tf::StampedTransform(
          tf_transform, ros::Time::now(), parent_frames_[x.first], x.first));
    }

    ros::WallDuration(0.02).sleep();
    ros::spinOnce();
  }
}

void ObjectServer::markerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (!do_update_)
  {
    ROS_WARN_THROTTLE(10, "Object server is set to paused.");
    return;
  }

  if (object_poses_.find(feedback->marker_name) != object_poses_.end())
  {
    object_poses_[feedback->marker_name] = feedback->pose;
  }
  else
  {
    ROS_WARN_STREAM("Got marker feedback for unknown marker "
                    << feedback->marker_name);
  }
}

void ObjectServer::setupMarker(
    visualization_msgs::InteractiveMarker &marker) const
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
  box_control.markers.push_back(box_marker);

  marker.controls.push_back(box_control);

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_server");
  ObjectServer server;
  server.runServer();
  return 0;
}
