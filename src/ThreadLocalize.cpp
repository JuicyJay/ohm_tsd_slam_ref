/*
 * ThreadLocalize.cpp
 *
 *  Created on: Nov 6, 2018
 *      Refactured by: jasmin
 */

#include "ThreadLocalize.h"
#include "SlamNode.h"
#include "ThreadMapping.h"
#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Logger.h"
#include <boost/bind.hpp>
#include <cstring>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

///whats that
//#define TRACE

namespace ohm_tsd_slam_ref
{

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle* nh, std::string nameSpace,
								const double xOffset, const double yOffset):
										ThreadSLAM(*grid),
										_nh(nh),
										_mapper(*mapper),
										_sensor(NULL),		///@todo nullptr?
										_initialized(false),
										_gridWidth(grid->getCellsX() * grid->getCellSize()),
										_gridHeight(grid->getCellsY() * grid->getCellSize()),
										_gridOffSetX(-1.0 * (grid->getCellsX() * grid->getCellSize() * 0.5 + xOffset)),
										_gridOffSetY(-1.0 * (grid->getCellsY() * grid->getCellSize() * 0.5 + yOffset)),
										_xOffset(xOffset),
										_yOffset(yOffset),
										_nameSpace(nameSpace),
										_stampLaser(ros::Time::now())
{
	ThreadLocalize* threadLocalize = NULL;		///@todo nullptr?

	double distFilterMax  			= 0.0;
	double distFilterMin			= 0.0;
	int icpIterations				= 0;
	std::string poseTopic;
	double durationWaitForOdom		= 0.0;
	///RandomMatcher options
	int trials						= 0;
	int sizeControlSet				= 0;
	double epsThresh				= 0.0;
	double zhit						= 0.0;
	double zphi						= 0.0;
	double zshort					= 0.0;
	double zmax						= 0.0;
	double zrand					= 0.0;
	double percentagePointsInC		= 0.0;
	double rangemax					= 0.0;
	double sigphi					= 0.0;
	double sighit					= 0.0;
	double lamshort					= 0.0;
	double maxAngleDiff				= 0.0;
	double maxAnglePenalty			= 0.0;
	int paramInt					= 0;
	int iVar 						= 0;

	/*** Read parameters from ros parameter server. Use namespace if provided. Multirobot use. ***/
	_nameSpace = nameSpace;
	std::string::iterator it = _nameSpace.end() - 1;		//stores last symbol of nameSpace
	if(*it != '/' && _nameSpace.size()>0)
		_nameSpace += "/";
	//pose
	poseTopic = _nameSpace + poseTopic;

	ros::NodeHandle prvNh("~");
	//ICP Options
	prvNh.param<double>(_nameSpace + "dist_filter_max", distFilterMax, DIST_FILT_MAX);
	prvNh.param<double>(_nameSpace + "dist_filter_min", distFilterMin, DIST_FILT_MIN);
	prvNh.param<int>(_nameSpace + "icp_iterations", icpIterations, ICP_ITERATIONS);

	prvNh.param(_nameSpace + "pose_topic", poseTopic, std::string("default_ns/pose"));
	prvNh.param("tf_base_frame", _tfBaseFrameId, std::string("/map"));
	prvNh.param(_nameSpace + "tf_child_frame", _tfChildFrameId, std::string("default_ns/laser"));
	prvNh.param("tf_odom_frame", _tfOdomFrameId, std::string("wheelodom"));
	prvNh.param("tf_footprint_frame", _tfFootprintFrameId, std::string("base_footprint"));
	prvNh.param<double>("reg_trs_max", _trnsMax, TRNS_THRESH);
	prvNh.param<double>("reg_sin_rot_max", _rotMax, ROT_THRESH);
	prvNh.param<double>("max_velocity_lin", _trnsVelocityMax, TRNS_VEL_MAX);
	prvNh.param<double>("max_velocity_rot", _rotVelocityMax, ROT_VEL_MAX);
	prvNh.param<bool>("ude_odom_rescue", _useOdomRescue, false);
	prvNh.param<double>("wait_for_odom_tf", durationWaitForOdom, 1.0);
	prvNh.param<double>("laser_min_range", _lasMinRange, 0.0);
	prvNh.param<int>("trials", trials, 100);
	prvNh.param<int>("sizeControlSet", sizeControlSet, 140);
	prvNh.param<double>("epsThresh", epsThresh, 0.15);
	prvNh.param<double>("zhit", zhit, 0.45);
	prvNh.param<double>("zphi", zphi, 0);
	prvNh.param<double>("zshort", zshort, 0.25);
	prvNh.param<double>("zmax", zmax, 0.05);
	prvNh.param<double>("zrand", zrand, 0.25);
	prvNh.param<double>("percentagePointsInC", percentagePointsInC, 0.9);
	prvNh.param<double>("rangemax", rangemax, 20);
	prvNh.param<double>("sigphi", sigphi, M_PI / 180.0 * 3);
	prvNh.param<double>("sighit", sighit, 0.2);
	prvNh.param<double>("lamshort", lamshort, 0.08);
	prvNh.param<double>("maxAngleDiff", maxAngleDiff, 3.0);
	prvNh.param<double>("maxAnglePenalty", maxAnglePenalty, 0.5);
	prvNh.param<int>(nameSpace + "ransac_trials", paramInt, RANSAC_TRIALS);						///@todo WARUM HIER nameSpace und nicht _nameSpace
	_ranTrials = static_cast<unsigned int>(paramInt);
	prvNh.param<double>(nameSpace + "ransac_eps_thresh", _ranEpsThresh, RANSAC_EPS_THRESH);
	prvNh.param<int>(nameSpace + "ransac_ctrlset_size", paramInt, RANSAC_CTRL_SET_SIZE);
	_ranSizeCtrlSet = static_cast<unsigned int>(paramInt);
	prvNh.param<double>(_nameSpace + "ransac_phi_max", _ranPhiMax, 30.0);
	prvNh.param<int>(_nameSpace + "registration_mode", iVar, ICP);

	///Align laserscans
	switch(_regMode)
	{
	case ICP:
		//no instance needed
		break;
	case EXP:
		_RandomNormalMatcher = new obvious::RandomNormalMatching(trials, epsThresh, sizeControlSet);
		break;
	case PDF:
		_PDFMatcher = new obvious::PDFMatching(trials, epsThresh, sizeControlSet, zhit, zphi, zshort, zmax, zrand, percentagePointsInC, rangemax,
												sigphi, sighit, lamshort, maxAngleDiff, maxAnglePenalty);
		break;
	case TSD:
		_TSD_PDFMatcher = new obvious::TSD_PDFMatching(_grid, trials, epsThresh, sizeControlSet, zrand);
		break;
	default:
		ROS_ERROR_STREAM("Unknown registration mode " << _regMode << " use default = ICP." << std::endl);
	}


	_odomAnalyzer 			= NULL;			///@todo nullptr? unten auch?
	_waitForOdomTf 			= ros::Duration(durationWaitForOdom);
	_odomTfIsValid 			= false;
	_regMode 				= static_cast<EnumRegModes>(iVar);
	_modelCoords			= NULL;
	_modelNormals			= NULL;
	_maskM					= NULL;
	_rayCaster				= NULL;			///@todo wofür wird der genullt? wird gleich hier unten initialisiert
	_scene					= NULL;
	_maskS					= NULL;
	_lastPose				= new obvious::Matrix(3, 3);
	_rayCaster				= new obvious::RayCastPolar2D();
	_assigner				= new obvious::FlannPairAssignment(2);
	_filterDist				= new obvious::DistanceFilter(distFilterMax, distFilterMin, icpIterations - 10);
	_filterReciprocal		= new obvious::ReciprocalFilter();
	_estimator 				= new obvious::ClosedFormEstimator2D();




ThreadLocalize::~ThreadLocalize() {
	// TODO Auto-generated destructor stub
}

} /* namespace ohm_tsd_slam_ref */
