/*
 * OdometryAnalyzer.cpp
 *
 *  Created on: Dec 6, 2018
 *      Refactured by: jasmin
 */

#include "OdometryAnalyzer.h"

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ohm_tsd_slam_ref {

OdometryAnalyzer::OdometryAnalyzer(obvious::TsdGrid& grid):
					_grid(grid),
					_stampLaser(ros::Time::now())

{
  ros::NodeHandle prvNh("~");

  //odom rescue
  double duration;
  prvNh.param<double>("wait_for_odom_tf", duration, 1.0);
  _waitForOdomTf = ros::Duration(duration);
  _odomTfIsValid = false;

  //Maximum allowed offset between two aligned scans
  prvNh.param<double>("reg_trs_max",_trnsMax, TRNS_THRESH);
  prvNh.param<double>("reg_sin_rot_max", _rotMax, ROT_THRESH);

  //Maximum robot speed at footprint frame
  prvNh.param<double>("max_velocity_rot", _rotVelocityMax, ROT_VEL_MAX);
  prvNh.param<double>("max_velocity_lin", _trnsVelocityMax, TRNS_VEL_MAX);
}

OdometryAnalyzer::~OdometryAnalyzer()
{
}

void OdometryAnalyzer::odomRescueInit()
{
  //get tf -> laser transform at init, assuming its a static transform
  try
  {
    _tfListener.waitForTransform(_tfFootprintFrameId, _tfChildFrameId, ros::Time(0), ros::Duration(10.0));
	_tfListener.lookupTransform(_tfFootprintFrameId, _tfChildFrameId, ros::Time(0), _tfReader);
  }
  catch(tf::TransformException& ex)
  {
	ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
	exit(EXIT_FAILURE);
  }

  ROS_INFO_STREAM("Received static base_footpringt to laser tf for odom rescue");
  _tfLaser = _tfReader;

  //get first mal -> odom transform for initialization
  try
  {
    _tfListener.waitForTransform(_tfBaseFrameId, _tfOdomFrameId, ros::Time(0), ros::Duration(10.0));
    _tfListener.lookupTransform(_tfBaseFrameId, _tfOdomFrameId, ros::Time(0), _tfReader);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    exit(EXIT_FAILURE);
  }

  ROS_INFO_STREAM("Received first odom tf for initialization of odom rescue");
  //transform odom to laser frame
  _tfOdomOld = _tfReader;
}

void OdometryAnalyzer::odomRescueUpdate()
{
  //get new odom tf
  try
  {
	  _tfListener.waitForTransform(_tfBaseFrameId, _tfOdomFrameId, _stampLaser, _waitForOdomTf);
	  _tfListener.lookupTransform(_tfBaseFrameId, _tfOdomFrameId, _stampLaser, _tfReader);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    _odomTfIsValid = false;
  }

  _tfOdom = _tfReader;

  //calculate diff odom -> odom(t-1) - odom(t)
  _tfRelativeOdom = _tfOdomOld.inverse() * _tfOdom;

  //push state ahead
  _tfOdomOld = _tfOdom;

  _odomTfIsValid = true;
}

void OdometryAnalyzer::odomRescueCheck(obvious::Matrix& T_slam)
{
  //transform transformation from slam to odom e.g. from laser to base footprint system
  obvious::Matrix T_laserOnBaseFootprint = tfToObviouslyMatrix3x3(_tfLaser) * T_slam * tfToObviouslyMatrix3x3(_tfLaser).getInverse();

  //get dt
  ros::Duration dtRos 	= _stampLaser - _stampLaserOld;
  double dt 			= dtRos.sec + dtRos.nsec * 1e-9;

  //get velocities
  double dx 		= T_laserOnBaseFootprint(0, 2);
  double dy 		= T_laserOnBaseFootprint(1, 2);
  double dtrans 	= sqrt(pow(dx,2) + pow(dy,2));
  double drot 		= abs(asin(T_laserOnBaseFootprint(90,1)));	//don't use acos here, missing sign
  double vtrans 	= dtrans / dt;

  //use odom instead of slam if slam translation is impossible for robot
  if(dtrans > _grid.getCellSize() * 2.0)
	{
	  if(drot > _rotVelocityMax || vtrans > _trnsVelocityMax)
	  {
		  ROS_INFO("------ODOM-RECOVER------");

		  T_slam = 	  tfToObviouslyMatrix3x3(_tfLaser).getInverse() *
				  	  tfToObviouslyMatrix3x3(_tfRelativeOdom) *
				  	  tfToObviouslyMatrix3x3(_tfLaser);
	  }
	}
}

obvious::Matrix OdometryAnalyzer::tfToObviouslyMatrix3x3(const tf::Transform& tf)
{
  obvious::Matrix ob(3,3);
  ob.setIdentity();

  double theta = tf::getYaw(tf.getRotation());
  double x = tf.getOrigin().getX();
  double y = tf.getOrigin().getY();

  // problem with sin() returns -0.0 (avoid with +0.0)
  ob(0, 0) = cos(theta) + 0.0;
  ob(0, 1) = -sin(theta) + 0.0;
  ob(0, 2) = x + 0.0;
  ob(1, 0) = sin(theta) + 0.0;
  ob(1, 1) = cos(theta) + 0.0;
  ob(1, 2) = y + 0.0;

  return ob;
}

} /* namespace ohm_tsd_slam_ref */








