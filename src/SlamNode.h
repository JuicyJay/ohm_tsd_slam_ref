/*
 * SlamNode.h
 *
 *  Created on: Oct 17, 2018
 *      Refactured by: jasmin
 */

#ifndef SLAMNODE_H_
#define SLAMNODE_H_

#include <ros/ros.h>
#include <vector>

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obcore/base/Logger.h"

#include "ThreadLocalize.h"
#include "ohm_tsd_slam_ref/StartStopSLAM.h"

/**
 * @brief INIT_PSHS number of initial pushes into the grid
 */
#define INIT_PSHS 1
/**
 * @brief THREAD_TERM_MS amount of time (ms) waiting for thread to terminate
 * @todo in ThreadGrid wird im Destruktor thread::join() aus boost aufgerufen, um auf Beendigung des Threads zu warten. SlamNode erbt aber nicht von ThreadSLAM, also besitzt es auch kein boost object thread. wäre es denkbar das auch damit zu lösen anstatt diese Zeit hier festzusetzen?
 */
#define THREAD_TERM_MS 1

/**
 * @todo ns später ändern
 */
namespace ohm_tsd_slam_ref
{

class ThreadSLAM;
class ThreadMapping;
class ThreadGrid;

/**
 * @brief TaggedSubscriber struct creates new datatype of subscribers for ros communication
 * @todo nachfragen
 */
struct TaggedSubscriber
{
	TaggesSubscriber(std::string topic, ThreadLocalize& localizer, ros::NodeHandle& nh):
		_topic(topic),
		_localizer(&localizer),
		_nh(&nh)
	{}
	TaggedSubscriber(const TaggedSubscriber& subs):
		_subs(subs._subs),
		_topic(subs._topic),
		_localizer(subs._localizer),
		_nh(subs._nh)
	{}
	TaggedSubscriber(void):
		_localizer(NULL),
		_nh(NULL)
	{}
	void switchOn(void)
	{
		_subs = _nh->subscribe(_topic, 1, &ThreadLocalize::laserCallBack, _localizer);
	}
	void switchOff(void)
	{
		_subs.shutdown();
	}
	///@todo was macht bool topic fct?
	bool topic(const std::string topic)
	{
		return topic == _topic;
	}

	ros::Subscriber _subs;
	std::string _topic;
	ThreadLocalize* _localizer;
	ros::NodeHandle _nh;
};

/**
 * @class SlamNode
 * @brief main node management of 2D slam, contains sub classes and ros interface
 * @author
 */
class SlamNode
{
public:
	/**
	 * @brief Constructor
	 * @brief class can be called in 2 modes, runSlam or runLocalize (run method invoked by start method)
	 * @brief default mode: SLAM mode, generates new empty obvious::TsdGrid, dimension and resolution can be set by defines in SlamNode.h
	 * @brief LOCALIZE mode: loads content from file at a given path, operates on a fixed model;
	 * in the fixed model no push into the grid is needed, no thread mapping (?richtig?) object is instantiated
	 */
	SlamNode(void);

	/**
	 * Destructor
	 * stop all localization threads
	 */
	virtual ~SlamNode();

	/**
	 * called in main function to start SlamNode
	 * calls run() method which defines the mode, SLAM or LOCALIZE
	 */
	void start(void){this->run();}

private:
	/**
	 * main slam method including ros::spinOnce etc.
	 * calls timedGridPub()
	 */
	void run(void);
	/**
	 * calculates time difference between last map and now
	 * calls unblock() method of class ThreadSLAM - to enable occupancy grid thread with
	 * certain frequency if time difference > _gridInterval ("thresh" - rate used for occupancy grid generation)
	 */
	void timedGridPub(void);
	/**
	 *
	 * @param req ROS service request
	 * @param res ROS service response
	 * @brief ROS service used for tilt scanner - stops SLAM when tilt movement is ongoing
	 * @return
	 * @todo nachfragen
	 */
	bool callBackServiceStartStopSLAM(ohm_tsd_slam_ref::StartStopSLAM::Request& req, ohm_tsd_slam_ref::StartStopSLAM::Response& res);
	/**
	 * Main node handle
	 */
	ros::NodeHandle _nh;
	/**
	 * Representation - instance of one TsdGrid
	 */
	obvious::TsdGrid* _grid;
	/**
	 * Create Mapping thread instance of class ThreadMapping
	 */
	ThreadMapping* _threadMapping;
	/**
	 * Grid thread instance of class ThreadGrid
	 */
	ThreadGrid* _threadGrid;
	/**
	 * X starting offset factor
	 * @todo same as _xOffFactor - prüfen, glaub das wird nicht verwendet -- launch file params auch löschen
	 */
	double _xOffFactor;
	/**
	 * Y starting offset factor
	 * @todo wofür gibt es die offset factors? im slamnode.cpp definier ich ja nochmal launch file params für xOffset u yOffset - sehe keine Verwendung von _yOffFactor
	 * @todo prüfen, glaub das wird nicht verwendet -- launch file params auch löschen
	 */
	double _yOffFactor;
	/**
	 * Rate used for occupancy grid generation
	 * Minimum time interval for new grid pub - if time between last pub and now is greater than _gridInterval - new occ grid pub
	 */
	ros::Duration* _gridInterval;
	/**
	 * Desired loop rate
	 */
	ros::Rate* _loopRate;
	/**
	 * ROS laser subscriber, storing all laser beams in a vector
	 */
	std::vector<TaggedSubscriber> _subsLaser;
	/**
	 * @todo was macht das hier?
	 * storing _grid, _threadMapping, xOffset and yOffset of ThreadLocalize
	 */
	std::vector<ThreadLocalize*> _localizers;
	/**
	 * @todo wofür brauchen wir den service?
	 */
	ros::ServiceServer _serviceStartStopSLAM;
};

} /* namespace ohm_tsd_slam_ref */

#endif /* SLAMNODE_H_ */
