/*
 * Odometry.h
 *
 *  Created on: Sep 5, 2022
 *      Author: francois
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <mutex>

#include "DataTypes.h"
#include "Log.h"

class Odometry {
private :
	std::mutex data_mutex;
	PositionSource position_source;
	Position current_position;
	Position lidar_position;
	Position odometry_position;

	Odometry();
	~Odometry();

	void _updateLidarPosition(Position *_new);
	void _updateOdometryPosition(Position *_new);
	void _setCurrentPositionSource(PositionSource new_source);
	void _updateCurrentPositionWithSource(PositionSource new_source);
	void _getCurrentPosition(Position *dest);

public :



	static Odometry& getInstance(void)
	{
		static Odometry odometry;
		return odometry;
	}

	static void copyPosition(Position *src, Position *dest);

	static void updateLidarPosition(Position *_new){
		getInstance()._updateLidarPosition(_new);
	}
	static void updateOdometryPosition(Position *_new){
		getInstance()._updateOdometryPosition(_new);
	}
	static void setCurrentPositionSource(PositionSource new_source){
		getInstance()._setCurrentPositionSource(new_source);
	}
	static void updateCurrentPositionWithSource(PositionSource new_source){
		getInstance()._updateCurrentPositionWithSource(new_source);
	}
	static void getCurrentPosition(Position *dest){
		getInstance()._getCurrentPosition(dest);
	}
	static void log(void){
		Position pos;
		getCurrentPosition(&pos);
		LOGF_TRACE("x=%.2f y=%.2f r=%.1f", pos.x, pos.y, pos.rot*180/3.1415);
	}
	static void logPos(Position pos){
		LOGF_TRACE("x=%.2f y=%.2f r=%.1f", pos.x, pos.y, pos.rot*180/3.1415);
	}

};




#endif /* ODOMETRY_H_ */
