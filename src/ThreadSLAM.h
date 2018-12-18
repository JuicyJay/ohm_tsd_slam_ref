/*
 * ThreadSLAM.h
 *
 *  Created on: Oct 30, 2018
 *      Refactured by: jasmin
 */

#ifndef THREADSLAM_H_
#define THREADSLAM_H_

#include <boost/thread.hpp>
#include "obvision/reconstruct/grid/TsdGrid.h"
#include <ros/ros.h>

namespace ohm_tsd_slam_ref
{

/**
 * @class ThreadSLAM
 * @brief Base class implementing boost thread functionality
 * @author Philipp Koch, Stefan May
 */
class ThreadSLAM
{
public:
	/**
	 * Constructor
	 * @param: reference to single tsdgrid _grid instantiated in SlamNode to pass it on to the other threads
	 */
	ThreadSLAM(obvious::TsdGrid& grid);

	/**
	 * Destructor
	 */
	virtual ~ThreadSLAM();

	/**
	 * Method to set a thread from sleep mode to run mode
	 */
	void unblock(void);

	/**
	 * Method to determine the state of a thread. Function tries to call the thread until the given time in ms runs out
	 * @param ms time trying to call thread
	 * @return true if thread could be called successfully during given ms
	 */
	bool alive(unsigned int ms);

	/**
	 * Method to terminate the thread
	 */
	void terminateThread(void);

protected:

	/**
	 * Abstract method connected to Boost threading functionality
	 *
	 */
	virtual void eventLoop(void) = 0;

	/**
	 * Boost threading object
	 */
	boost::thread* _thread;
	/**
	 * Boost sleeping mutex
	 */
	boost::mutex _sleepMutex;
	/**
	 * Boost condition variable for sleeping mode
	 */
	boost::condition_variable_any _sleepCond;
	/**
	 * Shutdown flag
	 */
	bool _stayActive;
	/**
	 * Reference to tsdgrid representation
	 */
	obvious::TsdGrid& _grid;

};

} /* namespace ohm_tsd_slam_ref */
#endif /* THREADSLAM_H_ */
