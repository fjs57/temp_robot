/*
 * Odometry.cpp
 *
 *  Created on: Sep 5, 2022
 *      Author: francois
 */

#include <string.h>

#include "Odometry.h"
#include "config.h"

Odometry::Odometry()
{
	data_mutex.lock();
	position_source = ODOMETRY_DEFAULT_POSITION_SOURCE;
	data_mutex.unlock();
}

Odometry::~Odometry()
{
	data_mutex.unlock();
}

void Odometry::copyPosition(Position *src, Position *dest)
{
	memcpy(dest, src, sizeof(Position));
}

void Odometry::_updateLidarPosition(Position *_new)
{
	data_mutex.lock();
	copyPosition(_new, &lidar_position);
	data_mutex.unlock();
	_updateCurrentPositionWithSource(position_source);
}

void Odometry::_updateOdometryPosition(Position *_new)
{
	data_mutex.lock();
	copyPosition(_new, &odometry_position);
	data_mutex.unlock();
	_updateCurrentPositionWithSource(position_source);
}

void Odometry::_updateCurrentPositionWithSource(PositionSource new_source)
{
	Position *source;
	switch (new_source)
	{
	case PositionSourceOdometry :
		source = &odometry_position;
		break;
	case PositionSourceLidar :
		source = &lidar_position;
		break;
	}
	data_mutex.lock();
	copyPosition(source, &current_position);
	data_mutex.unlock();
}

void Odometry::_setCurrentPositionSource(PositionSource new_source)
{
	data_mutex.lock();
	position_source = new_source;
	data_mutex.unlock();
}

void Odometry::_getCurrentPosition(Position *dest)
{
	data_mutex.lock();
	copyPosition(&current_position, dest);
	data_mutex.unlock();
}


