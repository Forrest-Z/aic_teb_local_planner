/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/LinearMath/Transform.h>
using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
ros::Subscriber init_pose_sub;
ros::Subscriber goal_pose_sub;
ros::Subscriber odom_sub;
ros::Subscriber scan_sub;
ros::Publisher velocity_pub;
std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;
tf::TransformListener* p_tf_listener;
sensor_msgs::LaserScan scan;
bool scan_updated;
// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent& e);
void CB_publishCycle(const ros::TimerEvent& e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);
void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id);
void CB_goal_pose(const geometry_msgs::PoseStampedConstPtr& msg);
void CB_odom(const nav_msgs::Odometry::ConstPtr& msg);
void scan_callback(const sensor_msgs::LaserScanConstPtr& msg);
void scan_to_obs(std::vector<ObstaclePtr> &obst_vec);
// =========== user qihao =============
PoseSE2 initial_pos(-4,0,0);
PoseSE2 goal_pos(4,0,0);
geometry_msgs::Twist g_cmd_vel;
nav_msgs::Odometry robot_odom;
geometry_msgs::Polygon footprint;
// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");
 
  tf::TransformListener tf_listener;
  p_tf_listener = &tf_listener;
  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);
 
  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.05), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
  //initial pose;
  tf::StampedTransform map_foot_frame;
  p_tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
  p_tf_listener->lookupTransform("map", "base_link", ros::Time(0), map_foot_frame);
  initial_pos.x()  = map_foot_frame.getOrigin().getX();
  initial_pos.y()  = map_foot_frame.getOrigin().getY();
  initial_pos.theta()  = tf::getYaw(map_foot_frame.getRotation());
  goal_pos.x()  = map_foot_frame.getOrigin().getX();
  goal_pos.y()  = map_foot_frame.getOrigin().getY();
  goal_pos.theta()  = tf::getYaw(map_foot_frame.getRotation());
  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);
  
  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("obstacles", 1, CB_customObstacle);

  scan_sub = n.subscribe("/scan", 1, scan_callback);

  //init pose and goal pose
  goal_pose_sub = n.subscribe("/move_base_simple/goal", 1, CB_goal_pose);

  odom_sub = n.subscribe("/odom", 1, CB_odom);

  velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  
  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));
  
  // Setup robot shape model
  RobotFootprintModelPtr robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n);

  planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));
  

  no_fixed_obstacles = obst_vector.size();
  ros::spin();

  return 0;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent& e)
{
   /***/
  try
  {
    tf::StampedTransform map_foot_frame;
    p_tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
    p_tf_listener->lookupTransform("map", "base_link", ros::Time(0), map_foot_frame);
    initial_pos.x()  = map_foot_frame.getOrigin().getX();
    initial_pos.y()  = map_foot_frame.getOrigin().getY();
    initial_pos.theta()  = tf::getYaw(map_foot_frame.getRotation());
  }
  catch (tf::TransformException ex)
  {
    initial_pos.x()  = 0;
    initial_pos.y()  = 0;
    initial_pos.theta()  = 0;
  }
  // via points
  //insert via point
  tf::Quaternion q;
  q.setRPY(0, 0, goal_pos.theta());
  tf::Transform map_goal(q, tf::Vector3(goal_pos.x(), goal_pos.y(), 0));
  q.setRPY(0, 0, 0);
  tf::Transform goal_viapoint_frame(q, tf::Vector3(0, 0, 0));
  tf::Transform map_viapoint_frame(q, tf::Vector3(0, 0, 0));
  tf::Transform map_initial(q, tf::Vector3(initial_pos.x(), initial_pos.y(), 0));
  tf::Transform goal_initial_frame   = map_goal.inverse()*map_initial;
  via_points.clear();
  double param_ds = 0.1;
  double param_via_x = 2;
  double tmp = goal_viapoint_frame.getOrigin().getX();
  while ((tmp+param_ds<param_via_x&&fabs(goal_initial_frame.getOrigin().getY())>param_ds)||
          (tmp+param_ds<goal_initial_frame.getOrigin().getX()&&fabs(goal_initial_frame.getOrigin().getY())<param_ds))
  {
    tmp  += param_ds;
    goal_viapoint_frame.getOrigin().setX(tmp);
    map_viapoint_frame            = map_goal*goal_viapoint_frame;
    via_points.emplace_back(map_viapoint_frame.getOrigin().x(), map_viapoint_frame.getOrigin().y());
  }
  tmp  = goal_viapoint_frame.getOrigin().getY();
  while( fabs(tmp)+param_ds<fabs(goal_initial_frame.getOrigin().getY()))
  {
    if (goal_initial_frame.getOrigin().getY()>0)
    {
      tmp  +=param_ds;
      goal_viapoint_frame.getOrigin().setY(tmp);
    }
    else
    {
      tmp  -=param_ds;
      goal_viapoint_frame.getOrigin().setY(tmp);
    }
    map_viapoint_frame            = map_goal*goal_viapoint_frame;
    via_points.emplace_back(map_viapoint_frame.getOrigin().x(), map_viapoint_frame.getOrigin().y());
  }
  //teb local planner
  planner->plan(initial_pos, goal_pos,&robot_odom.twist.twist,false); // hardcoded start and goal for testing purposes

  if(fabs(initial_pos.x() - goal_pos.x())+fabs(initial_pos.y() - goal_pos.y())>0.01)
  {
    //publish vel cmd
    geometry_msgs::Twist cmd_vel;
    double vy;
    planner.get()->getVelocityCommand(cmd_vel.linear.x, vy, cmd_vel.angular.z,1);
    velocity_pub.publish(cmd_vel);
  }
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent& e)
{
  planner->visualize();//publish poses(path+time) and path 
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( box_control );

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name,feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;
  
  if (index>=no_fixed_obstacles) 
    return;
  PointObstacle* pobst = static_cast<PointObstacle*>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);	  
}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);
  
  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
  {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1 )
    {
      if (obst_msg->obstacles.at(i).radius == 0) 
      {
        obst_vector.push_back(ObstaclePtr(new PointObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                           obst_msg->obstacles.at(i).polygon.points.front().y )));
      }
      else
      {
        obst_vector.push_back(ObstaclePtr(new CircularObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y,
                                                            obst_msg->obstacles.at(i).radius )));
      }
    }
    else
    {
      PolygonObstacle* polyobst = new PolygonObstacle;
      for (size_t j=0; j<obst_msg->obstacles.at(i).polygon.points.size(); ++j)
      {
        polyobst->pushBackVertex( obst_msg->obstacles.at(i).polygon.points[j].x,
                                  obst_msg->obstacles.at(i).polygon.points[j].y );
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }

    if(!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
  }
}

void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}

void CB_goal_pose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  goal_pos.x()                       = msg->pose.position.x;
  goal_pos.y()                       = msg->pose.position.y;
  goal_pos.theta()                   = tf::getYaw(msg->pose.orientation);
}
void CB_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_odom.twist.twist.linear.x     = msg->twist.twist.linear.x;
  robot_odom.twist.twist.linear.y     = msg->twist.twist.linear.y;
  robot_odom.twist.twist.angular.z    = msg->twist.twist.angular.z;
}

void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id)
{
  if (id >= obst_vector.size())
  {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel (twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}

void scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  scan = *msg;
  scan_updated = true;
  obst_vector.clear();
  //scan_to_obs(obst_vector);

  //insert vitual linear obstacles
  tf::Quaternion q;
  q.setRPY(0, 0, goal_pos.theta());
  tf::Transform map_goal(q, tf::Vector3(goal_pos.x(), goal_pos.y(), 0));
  q.setRPY(0, 0, 0);
  tf::Transform goal_tmp(q, tf::Vector3(-1, -0.5, 0));
  tf::Transform goal_tmp1(q, tf::Vector3(1, -0.5, 0));
  goal_tmp            = map_goal*goal_tmp;
  goal_tmp1           = map_goal*goal_tmp1;
  obst_vector.push_back( boost::make_shared<LineObstacle>(goal_tmp.getOrigin().x(), goal_tmp.getOrigin().y(),goal_tmp1.getOrigin().x(), goal_tmp1.getOrigin().y()));
  goal_tmp.setOrigin(tf::Vector3(-1, 0.5, 0));
  goal_tmp.setRotation(q);
  goal_tmp1.setOrigin(tf::Vector3(1, 0.5, 0));
  goal_tmp1.setRotation(q);
  goal_tmp            = map_goal*goal_tmp;
  goal_tmp1           = map_goal*goal_tmp1;
  obst_vector.push_back( boost::make_shared<LineObstacle>(goal_tmp.getOrigin().x(), goal_tmp.getOrigin().y(),goal_tmp1.getOrigin().x(), goal_tmp1.getOrigin().y()));
}

void scan_to_obs(std::vector<ObstaclePtr> &obst_vec)
{
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tf::StampedTransform map_laser_frame;
  tf::Transform laser_obs_frame(q,tf::Vector3(0,0,0));
  p_tf_listener->lookupTransform("map", scan.header.frame_id, ros::Time(0), map_laser_frame);
  //foot_laser_frame = listener.lookupTransform("base_footprint", "laser_link", ros::Time(0));
  float angle = scan.angle_min;
  for(auto r : scan.ranges){
    float x = r * cos(angle);
    float y = r * sin(angle);
    laser_obs_frame.setOrigin(tf::Vector3(x,y,0));
    tf::Transform map_obs_frame = map_laser_frame * laser_obs_frame;
    x = map_obs_frame.getOrigin().getX();
    y = map_obs_frame.getOrigin().getY();
    angle += scan.angle_increment;
    obst_vec.push_back( boost::make_shared<PointObstacle>(x,y) );

  }
  return;
}