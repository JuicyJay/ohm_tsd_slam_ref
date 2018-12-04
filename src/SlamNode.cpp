/*
 * SlamNode.cpp
 *
 *  Created on: Oct 17, 2018
 *      Refactured by: jasmin
 */

#include "SlamNode.h"
#include "ThreadMapping.h"
#include "ThreadGrid.h"

#include "obcore/math/mathbase.h"

namespace ohm_tsd_slam_ref {

SlamNode::SlamNode(void)
{
	double loopRateVar 			= 0.0;
	double xOffset 				= 0.0;
	double yOffset 				= 0.0;
	double gridPublishInterval 	= 0.0;
	//integer variable, used for multiple launchfileparams, casts
	int iVar 					= 0;
	unsigned int robotNbr 		= static_cast<unsigned int>(iVar); 	//cast because no unsigned int in launch file
	std::string nameSpace		= "";
	std::string topicLaser;
	std::string topicServiceStartStop;

	ros::NodeHandle prvNh("~");
	prvNh.param<double>("loop_rate", loopRateVar, 40.0);
	prvNh.param<double>("x_offset", xOffset, 0.0);
	prvNh.param<double>("y_offset", yOffset, 0.0);
	prvNh.param<double>("occ_grid_time_interval", gridPublishInterval, 2.0);
	prvNh.param<int>("robot_nbr", iVar, 1);

	//PRÜFEN - glaub die werden nicht mehr verwendet
	prvNh.param<double>("x_off_factor", _xOffFactor, 0.5);
	prvNh.param<double>("y_off_factor", _yOffFactor, 0.5);

	_loopRate = new ros::Rate(loopRateVar);
	//instantiate Mapping Thread
	_threadMapping = new ThreadMapping(_grid);
	_threadGrid = new ThreadGrid(_grid, &_nh, xOffset, yOffset);
	_gridInterval = new ros::Duration(gridPublishInterval);

	ThreadLocalize* threadLocalize = NULL;

	TaggedSubscriber subs;

	//instantiate Localization Threads for single or multi slam
	//for single slam
	if(robotNbr == 1)
	{
		threadLocalize = new ThreadLocalize(_grid, _threadMapping, &_nh, nameSpace, xOffset, yOffset);
		subs = TaggedSubscriber(topicLaser, *threadLocalize, _nh);
		subs.switchOn();
		_subsLaser.push_back(subs);
		_localizers.push_back(threadLocalize);
		ROS_INFO_STREAM("Single SLAM started." << std::endl);
	}
	//for multi slam
	else
	{
		for(unsigned int i = 0; i < robotNbr; i++)	//looping through all active robots
		{
			std::stringstream sstream;
			sstream << "robot";
			sstream << i << "/namespace";
			///@todo verwirrend
			std::string dummy = sstream.str();
			prvNh.param(dummy, nameSpace, std::string("default_ns"));
			threadLocalize = new ThreadLocalize(_grid, _threadMapping, &_nh, nameSpace, xOffset, yOffset);
			subs = TaggedSubscriber(nameSpace + "/" + topicLaser, *threadLocalize, _nh);
			_subsLaser.push_back(subs);
			_localizers.push_back(threadLocalize);
			ROS_INFO_STREAM("started Localization Thread for " << nameSpace << std::endl);
		}
		ROS_INFO_STREAM("All Localization Threads instantiated, MULTI slam started." << std::endl);
	}
	_serviceStartStopSLAM = _nh.advertiseService(topicServiceStartStop, &SlamNode::callBackServiceStartStopSLAM, this);
}

SlamNode::~SlamNode()
{
	///@todo geht das so mit neuer foreach schleifn? prüfen, wenn ThreadLocalize erstellt ist
	//stop localize threads
	for(auto& i: _localizers)
	{
		i->terminateThread();
		while(i->alive(THREAD_TERM_MS))
			usleep(THREAD_TERM_MS);
		delete i;
		i++;
	}
	delete _loopRate;
	delete _gridInterval;
	//stop mapping threads
	_threadGrid->terminateThread();
	while(_threadGrid->alive(THREAD_TERM_MS))
		usleep(THREAD_TERM_MS);
	delete _threadGrid;
	_threadMapping->terminateThread();
	while(_threadMapping->alive(THREAD_TERM_MS))
		usleep(THREAD_TERM_MS);
	delete _threadMapping;
	delete _grid;
}

void SlamNode::run(void)
{
	ROS_INFO_STREAM("Waiting for first laser scan to initialize node ... \n");
	while(ros::ok())
	{
		ros::spinOnce();
		this->timedGridPub();
		_loopRate->sleep();
	}
}

void SlamNode::timedGridPub(void)
{
	static ros::Time lastMap = ros::Time::now();
	ros::Time curTime = ros::Time::now();

	if((curTime - lastMap).toSec() > _gridInterval->toSec())
	{
		_threadGrid->unblock();
		lastMap = ros::Time::now();
	}
}

bool SlamNode::callBackServiceStartStopSlam(ohm_tsd_slam_ref::StartStopSLAM::Request& req, ohm_tsd_slam_ref::StartStopSLAM::Response& res)
{
	///@todo hier nullptr?
	TaggedSubscriber* subsCur = NULL;
	///@todo foreach - wie geht das hier?
	///@todo und was passiert hier eigentlich
	for(auto& i: _subsLaser)
	{
		if(i->topic(req.topic))
			subsCur = &i;
	}
	if(!subsCur)
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << "Error! Topic " << req.topic << " invalid!");
		return false;
	}
	if(req.startStop == req.START)
	{
		ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "Started SLAM for topic " << req.topic);
		subsCur->switchOff();
	}
	else
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << "Error! Unknown request for service. ");
		return false;
	}
	return true;
}








} /* namespace ohm_tsd_slam_ref */
