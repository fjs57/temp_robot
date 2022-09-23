/*
 * Positionning.h
 *
 *  Created on: Sep 2, 2022
 *      Author: francois
 */

#ifndef POSITIONNING_H_
#define POSITIONNING_H_

#include <DataTypes.h>
#include <vector>
#include <mutex>
#include <thread>

#include "DBSCAN.h"
#include "config.h"

namespace positionning {
	typedef enum States_ {
		WAITING,
		START,
		RUNNING,
		COMPUTE,
		COMPUTE_WAIT
	} States;
}

class Positionning {
private :

	std::vector<Cluster> clusters;
	std::vector<Point> dataset;
	std::vector<Point> points;

	float x, y, rot;
	float error;
	std::mutex data_mutex;
	std::mutex clusters_mutex;

	DBSCAN algo;

	int sampling_repetition;
	int file_index;

	bool start;
	bool continuous;
	bool dataReady;

	positionning::States state;

	std::thread compute_thread;


	void addPoints(std::vector<Point> _points);
	void transformToCartesian(void);
	void clustering(void);
	void lowLevelFilter(void);
	void computePosition(void);

	float distanceBewteenClusters(Cluster a, Cluster b);
	float threeBeaconTest(Cluster a, Cluster b, Cluster c);

public :

	Positionning();
	~Positionning();

	void process(std::vector<Point> _points);

	void startMeasure(void);
	void startContinuousMeasure(void);
	void stopContinuousMeasure(void);


	bool isDataReady(void);


};



#endif /* POSITIONNING_H_ */
