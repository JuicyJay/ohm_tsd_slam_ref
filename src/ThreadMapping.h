/*
 * ThreadMapping.h
 *
 *  Created on: Oct 31, 2018
 *      Refactured by: jasmin
 */

#ifndef THREADMAPPING_H_
#define THREADMAPPING_H_

#include "ThreadSLAM.h"
#include "sensor_msgs/LaserScan.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include <boost/thread.hpp>
#include <deque>

namespace ohm_tsd_slam_ref
{

class SlamNode;

/**
 * @class ThreadMapping
 * @brief Implements a thread updating an obvious:TsdGrid
 * @author Philipp Koch, Stefan May
 */

class ThreadMapping : public ThreadSLAM
{
public:
	/**
	 * Constructor
	 * @param grid Representation taken over from base class ThreadSLAM
	 */
	ThreadMapping(obvious::TsdGrid* grid);

	/**
	 * Desctructor
	 */
	virtual ~ThreadMapping();
	/**
	 * Method to add an instance of SensorPolar2D to the queue
	 * called by ThreadLocalize if pose change is significant
	 * @param sensor new sensor data (pose, laser, mask..)
	 */
	void queuePush(obvious::SensorPolar2D* sensor);
	/**
	 * Bool determining whether TSD grid contains data or not, called by ThreadLocalize
	 * @return true in case of an initialized grid (contains data)
	 */
	bool initialized(void);

	/**
	 * Method to initialize the grid from a certain pose, called by ThreadLocalize
	 * @param sensor initial sensor data (pose, laser, mask..)
	 */
	void initPush(obvious::SensorPolar2D* sensor);

protected:
	/**
	 * abstract function derived from base class ThreadSLAM;
	 * consists of a loop the thread never leaves until a termination call occurs
	 * updates the representation with the first sensor element in the deque _sensors and removes used sensor element;
	 * iterates until queue is empty - enters blocked mode after that
	 */
	virtual void eventLoop(void);

private:
	/**
	 * Sensor double-ended-queue, allows fast insertion and deletion at its beginning and end
	 */
	std::deque<obvious::SensorPolar2D*> _sensors;
	/**
	 * Push mutex for double-ended-queue; synchronization necessary since queue is accessed by two threads
	 * method locks the mutex before adding data and unlocks it before terminating
	 */
	boost::mutex _pushMutex;
	/**
	 * Flag initialized
	 */
	bool _initialized;
};

} /* namespace ohm_tsd_slam_ref */

#endif /* THREADMAPPING_H_ */
