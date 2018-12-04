/*
 * ThreadMapping.cpp
 *
 *  Created on: Oct 31, 2018
 *      Refactured by: jasmin
 */

#include "ThreadMapping.h"
#include "SlamNode.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include <cmath>

namespace ohm_tsd_slam_ref
{

ThreadMapping::ThreadMapping(obvious::TsdGrid* grid):
		ThreadSLAM(*grid),
		_initialized(false)
{
}

ThreadMapping::~ThreadMapping()
{
	_thread->join();		//boost method to wait for a thread of execution to finish
}

void ThreadMapping::queuePush(obvious::SensorPolar2D* sensor)
{
	_pushMutex.lock();
	obvious::SensorPolar2D* sensorLocal = 	new obvious::SensorPolar2D(sensor->getRealMeasurementSize(), sensor->getAngularResolution(), sensor->getPhiMin(),
																		sensor->getMaximumRange(), sensor->getMinimumRange(), sensor->getLowReflectivityRange());
	sensorLocal->setTransformation(sensor->getTransformation());
	sensorLocal->setRealMeasurementData(sensor->getRealMeasurementData());
	sensorLocal->setStandardMask();
	_sensors.push_back(sensorLocal);
	_pushMutex.unlock();
	this->unblock();		//Method from ThreadSLAM to set a thread from sleep mode to run mode
}

bool ThreadMapping::initialized(void)
{
	bool var = false;
	_pushMutex.lock();
	var = _initialized;
	_pushMutex.unlock();
	return var;
}

void ThreadMapping::initPush(obvious::SensorPolar2D* sensor)
{
	if(this->initialized())
		return;
	_pushMutex.lock();
	for(unsigned int i = 0; i < INIT_PSHS; i++)			//INIT_PSHS nbr of initial pushed into the grid defined in SlamNode.h
		_grid.push(sensor);								//_grid instance inherited from ThreadSLAM
	_initialized = true;
	_pushMutex.unlock();
}

void ThreadMapping::eventLoop(void)
{
	while(_stayActive)
	{
		_sleepCond.wait(_sleepMutex);
		while(_stayActive && !_sensors.empty())
		{
			_pushMutex.lock();
			obvious::SensorPolar2D* sensor = _sensors.back();
			_sensors.pop_back();
			_pushMutex.unlock();

			_grid.push(sensor);
			_pushMutex.lock();
			delete sensor;
			_initialized = true;
			_pushMutex.unlock();
		}
	}
}

} /* namespace ohm_tsd_slam_ref */
