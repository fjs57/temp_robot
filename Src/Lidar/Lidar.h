/*
 * Lidar.h
 *
 *  Created on: Sep 1, 2022
 *      Author: francois
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <thread>
#include <mutex>

#include "CYdLidar.h"
#include "Dataset.h"

class Lidar {
private :
	CYdLidar laser;
	LaserScan scan;

	int file_index;

	Dataset dataset;
	std::mutex data_mutex;

	bool status;
	bool state;

	std::thread process_thread;
	bool process_state;


	void process(void);
	void dataProcess(void);

	void startThread(void);
	void stopThread(void);

	void computePosition(void);

public :
	Lidar();
	~Lidar();

	bool initLidar(void);

	bool start(void);
	void stop(void);

	bool getState(void){
		return laser.getIsScanning();
	}
};


#endif /* LIDAR_H_ */
