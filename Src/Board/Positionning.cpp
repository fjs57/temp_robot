/*
 * Positionning.cpp
 *
 *  Created on: Sep 2, 2022
 *      Author: francois
 */

#include <algorithm>
#include <cmath>
#include <numeric>
#include <fstream>
#include <iostream>
#include <random>

#include "Positionning.h"
#include "DBSCAN.h"
#include "Log.h"
#include "Odometry.h"

#include "ClientUDP.h"

using namespace std;

Positionning::Positionning()
{
	x = 0;
	y = 0;
	rot = 0;
	error = 0;
	file_index = 0;
	dataReady = false;
	sampling_repetition = 0;
	start = 0;
	state = positionning::WAITING;
	continuous = false;
}

void Positionning::addPoints(std::vector<Point> _points)
{
	points.insert(points.end(), _points.begin(), _points.end());
}

void Positionning::transformToCartesian(void)
{
	for(int i=0; i<(int)points.size(); i++)
	{
		points[i].x = points[i].range * cosf32((-1)*points[i].angle);
		points[i].y = points[i].range * sinf32((-1)*points[i].angle);
	}
}

void Positionning::clustering(void)
{

    Point p;
    dataset.clear();
    dataset = points;

    auto rng = default_random_engine{};
    shuffle(begin(dataset), end(dataset), rng);

    algo.m_points = dataset;
    algo.run();
    dataset = algo.m_points;


    clusters_mutex.lock();
    clusters.clear();

    for(int i=0; i<(int)dataset.size(); i++)
    {
    	p = dataset[i];

    	if(p.clusterID < 0) continue;

    	if (p.clusterID >= (int)clusters.size())
    	{
    		clusters.resize(p.clusterID+1);

    		clusters[p.clusterID].x = 0;
    		clusters[p.clusterID].y = 0;
    		clusters[p.clusterID].count = 0;
    		clusters[p.clusterID].max_x = p.x;
    		clusters[p.clusterID].min_x = p.x;
    		clusters[p.clusterID].max_y = p.y;
    		clusters[p.clusterID].min_y = p.y;

    	}

    	clusters[p.clusterID].x += p.x;
    	clusters[p.clusterID].y += p.y;

    	clusters[p.clusterID].max_x = max(clusters[p.clusterID].max_x, p.x);
    	clusters[p.clusterID].min_x = min(clusters[p.clusterID].min_x, p.x);
    	clusters[p.clusterID].max_y = max(clusters[p.clusterID].max_y, p.y);
    	clusters[p.clusterID].min_y = min(clusters[p.clusterID].min_y, p.y);

		clusters[p.clusterID].count++;

    }

    for(int i=1; i<(int)clusters.size(); i++)
    {
    	clusters[i].x /= (float)clusters[i].count;
    	clusters[i].y /= (float)clusters[i].count;
    	clusters[i].std_x = (clusters[i].max_x - clusters[i].min_x);
    	clusters[i].std_y = (clusters[i].max_y - clusters[i].min_y);
    	clusters[i].isBeacon = false;
    	clusters[i].id = i;
    }
    clusters_mutex.unlock();
}

void Positionning::lowLevelFilter(void)
{
	clusters_mutex.lock();
	clusters.erase(
        remove_if(
            clusters.begin(),
            clusters.end(),
            [](Cluster c){
				return (c.std_x > POSITIONNING_MAX_STDDEV) ||
						(c.std_y > POSITIONNING_MAX_STDDEV) ||
						(c.x == 0 && c.y == 0);
            }
        ),
        clusters.end()
    );
	clusters_mutex.unlock();
}

float Positionning::distanceBewteenClusters(Cluster a, Cluster b)
{
	return sqrtf32((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

void Positionning::computePosition(void)
{

	Cluster a, b, c;
	float best_error = -1;
	float current_error;
	Cluster ba, bb, bc;
//	int bi, bj, bk;

	float dx, dy, dz;
	Cluster cx, cy, cz;
	cx.x = BEACON_POSITION_A_X;
	cx.y = BEACON_POSITION_A_Y;
	cy.x = BEACON_POSITION_B_X;
	cy.y = BEACON_POSITION_B_Y;
	cz.x = BEACON_POSITION_C_X;
	cz.y = BEACON_POSITION_C_Y;
	dx = distanceBewteenClusters(cx,  cy);
	dy = distanceBewteenClusters(cy,  cz);
	dz = distanceBewteenClusters(cz,  cx);


	clusters_mutex.lock();


	float da, db, dc;

	for(int i=0; i<(int)clusters.size(); i++)
	{
		clusters[i].isBeacon = false;
		a = clusters[i];

		for(int j=0; j<(int)clusters.size(); j++)
		{
			if(i==j) continue;
			b = clusters[j];


			for(int k=0; k<(int)clusters.size(); k++)
			{
				if(k==j || k==i) continue;
				c = clusters[k];


				da = distanceBewteenClusters(a,  b) - dx;
				db = distanceBewteenClusters(b,  c) - dy;
				dc = distanceBewteenClusters(c,  a) - dz;


				current_error = sqrtf32(da*da+db*db+dc*dc);


				if(best_error > current_error || best_error == -1)
				{
					best_error = current_error;
					ba = a;
					bb = b;
					bc = c;
//					bi = i;
//					bj = j;
//					bk = k;
				}
			}
		}
	}


//	clusters[bi].isBeacon = true;
//
//	clusters[bj].isBeacon = true;
//
//	clusters[bk].isBeacon = true;


	clusters_mutex.unlock();



#ifdef POSITIONNING_MAX_ERROR
	if (best_error > POSITIONNING_MAX_ERROR){
#ifdef POSITIONNING_TRACE
		LOGF_TRACE("Ignored, error too big %.4f", best_error);
#endif
		return;
	}
#endif

	LOG_DEBUG("-");

	float _x, _y, _rot;

	Cluster PM;
	PM.x = (ba.x + bb.x)/2;
	PM.y = (ba.y + bb.y)/2;

	dx = bc.x - PM.x;
	dy = bc.y - PM.y;

	_rot =      atan2f32(dy, dx);

	PM.x = -(PM.x + bc.x)/2;
	PM.y = -(PM.y + bc.y)/2;

	_x = PM.x * cosf32(_rot) + PM.y * sinf32(_rot);
	_y = PM.x * (-sinf32(_rot)) + PM.y * cosf32(_rot);

	_rot *= -1;

	data_mutex.lock();
	x = _x;
	y = _y;
	rot = _rot;
	error = best_error;
	Position pos = {x,y,rot};
	data_mutex.unlock();
	Odometry::updateLidarPosition(&pos);
//	ClientUDP::sendDatagram("%.3f;%.3f;%.3f", pos.x, pos.y, pos.rot);

#ifdef POSITIONNING_TRACE
	LOGF_TRACE("x:%.2f y:%.2f r:%.1f° err:%.1fcm", _x, _y, _rot*180/(float)M_PI, best_error*100);
#endif
}

void Positionning::process(std::vector<Point> _points)
{
	switch(state)
	{
	case positionning::WAITING :
		LOG_DEBUG("state = WAITING");

		if(start || continuous){
			start = false;
			state = positionning::START;
		}

		break;

	case positionning::START :
		LOG_DEBUG("state = START");

		clusters_mutex.lock();
		clusters.clear();
		dataset.clear();
		points.clear();
		clusters_mutex.unlock();

		sampling_repetition = 0;

		/* no break */

	case positionning::RUNNING :
		if(state != positionning::RUNNING)
		{
			state = positionning::RUNNING;
		} else {
			LOG_DEBUG("state = RUNNING");
		}

		sampling_repetition++;

		addPoints(_points);

		if(sampling_repetition>=POSITIONNING_SAMPLING_REPETITIONS)
			state = positionning::COMPUTE;

		break;

	case positionning::COMPUTE :
		LOG_DEBUG("state = COMPUTE");

		start = false;
		dataReady = false;



		compute_thread = std::thread(
			[this](){
				this->transformToCartesian();
				this->clustering();
				this->lowLevelFilter();
				this->computePosition();
				dataReady = true;
			}
		);
		state = positionning::COMPUTE_WAIT;
		break;

	case positionning::COMPUTE_WAIT :
		LOG_DEBUG("state = COMPUTE WAIT");
		if(dataReady){
			state = positionning::WAITING;
			if(compute_thread.joinable()) compute_thread.join();

			if(continuous)
			{
				state = positionning::START;
			} else {
				state = positionning::WAITING;
			}
		}
		break;
	}
}

void Positionning::startMeasure(void)
{
	start = true;
}

bool Positionning::isDataReady(void)
{
	return dataReady;
}

void Positionning::startContinuousMeasure(void)
{
	continuous = true;
}

void Positionning::stopContinuousMeasure(void)
{
	continuous = false;
}

Positionning::~Positionning()
{
	data_mutex.unlock();
	clusters_mutex.unlock();
	if(compute_thread.joinable()) compute_thread.join();
}


