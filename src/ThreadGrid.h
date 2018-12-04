/*
 * ThreadGrid.h
 *
 *  Created on: Oct 30, 2018
 *      Refactured by: jasmin
 */

#ifndef THREADGRID_H_
#define THREADGRID_H_

#include "ThreadSLAM.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>

namespace ohm_tsd_slam_ref
{

/**
 * @class ThreadGrid
 * @brief Class implementing a thread that generates an occupancy grid
 * @author Philipp Koch, Stefan May
 */
class ThreadGrid : public ThreadSLAM
{
public:

	/**
	 * Constructor
	 * @param grid Representation
	 * @param nh ROS nodehandle
	 * @param xOffset
	 * @param yOffset
	 */
	ThreadGrid(obvious::TsdGrid* grid, ros::NodeHandle* const nh, const double xOffset, const double yOffset);

	/**
	 * Destructor
	 */
	virtual ~ThreadGrid();

protected:
	/**
	 * Event Loop for thread
	 * @todo können wir bitttttte die occupied cells berechnung mal durchgehen
	 */
	virtual void eventLoop(void);
private:
	/**
	 * ROS service callback method for GetMap service
	 * Get the map as a nav_msgs/OccupancyGrid
	 * @param req Request
	 * @param res Response
	 * @return true if successful
	 */
	bool getMapServCallBack(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);
	/**
	 * Occupancy grid
	 */
	nav_msgs::OccupancyGrid* _occGrid;
	/**
	 * ROS server for GetMap service
	 */
	ros::ServiceServer _getMapServ;
	/**
	 * Buffer for occupancy grid content
	 * @todo eventuell abändern, veraltet
	 */
	char* _occGridContent;
	/**
	 * Buffer for grid coordinates
	 */
	double* _gridCoords;
	/**
	 * Grid dimension
	 */
	unsigned int _width;
	/**
	 * Grid dimension
	 */
	unsigned int _height;
	/**
	 * Grid resolution
	 */
	double _cellSize;
	/**
	 * ROS occupancy grid publisher
	 */
	ros::Publisher _gridPub;
	/**
	 * ROS color image publisher
	 */
	ros::Publisher _pubColorImage;
	/**
	 * Publish color map control flag
	 */
	bool _pubTsdColorMap;
	/**
	 * Object inflation factor
	 */
	unsigned int _objInflateFactor;
	/**
	 * Object inflation control flag
	 */
	bool _objectInflation;
};

} /* namespace ohm_tsd_slam_ref */

#endif /* THREADGRID_H_ */
