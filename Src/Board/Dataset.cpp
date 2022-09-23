/*
 * Dataset.cpp
 *
 *  Created on: Sep 2, 2022
 *      Author: francois
 */

#include <cmath>
#include <iostream>
#include <fstream>
#include <numeric>

#include "Dataset.h"
#include "DataTypes.h"
#include "core/math/angles.h"
#include "Odometry.h"
#include "Terrain.h"

#include "ClientUDP.h"
#include "Log.h"
#include "Odometry.h"

#include "config.h"

using namespace std;
using namespace ydlidar::core::math;

Dataset::Dataset()
{

    protection_score = -1;
}

void Dataset::positionning_process(void)
{
	positionning.process(dataset);
}

void Dataset::positionning_start(void)
{
#ifdef POSITIONNING_CONTINUOUS_MEASURE
	positionning.startContinuousMeasure();
#else
	positionning.startMeasure();
#endif
}

void Dataset::process(LaserScan scan)
{
    construct_dataset(scan);
    low_level_filter();
    positionning_process();
    referential_transform();
    medium_level_filter();
    obstacle_detection();
    clustering();
    high_level_filter();
    send_points_with_clusters();
    Terrain::setObstacles(clusters);
    Terrain::computePath();
}

void Dataset::construct_dataset(LaserScan scan)
{
    Point pt;

    dataset.clear();

    for ( int i = 0; i < (int)scan.points.size(); i++ )
    {
        pt.angle = scan.points[i].angle;
        pt.range = scan.points[i].range;
        pt.x = 0;
        pt.y = 0;
        pt.z = 0;
        pt.clusterID = UNCLASSIFIED;

        dataset.push_back(pt);
    }
}

bool DatasetLowLevelFilter(Point pt)
{
    if ( pt.range <= LIDAR_MINIMUM_DETECTION_RADIUS ) return true;
    if ( pt.range >= LIDAR_DETECTION_RADIUS ) return true;
    return false;
}

void Dataset::low_level_filter(void)
{
	dataset.erase(
        remove_if(
            dataset.begin(),
            dataset.end(),
            DatasetLowLevelFilter
        ),
        dataset.end()
    );
}

void Dataset::referential_transform(void)
{
    float angle, range;

    Position robot;
    Odometry::getCurrentPosition(&robot);

    for ( int i = 0; i < (int)dataset.size(); i++ )
    {
        angle = - dataset[i].angle;
        range = dataset[i].range;

        dataset[i].x = range * cosf32(angle + robot.rot) + robot.x;
        dataset[i].y = range * sinf32(angle + robot.rot) + robot.y;
    }
}

bool DatasetMediumLevelFilter(Point pt)
{
    if ( pt.x > FIELD_BOUND_X_MAX - TERRAIN_FILTER_MARGIN ) return true;
    if ( pt.x < FIELD_BOUND_X_MIN + TERRAIN_FILTER_MARGIN ) return true;
    if ( pt.y > FIELD_BOUND_Y_MAX - TERRAIN_FILTER_MARGIN ) return true;
    if ( pt.y < FIELD_BOUND_Y_MIN + TERRAIN_FILTER_MARGIN ) return true;
    return false;
}

void Dataset::medium_level_filter(void)
{
        dataset.erase(
        remove_if(
            dataset.begin(),
            dataset.end(),
            DatasetMediumLevelFilter
        ),
        dataset.end()
    );
}

void Dataset::obstacle_detection(void)
{
    float radius, angle;
    protection_score = 0;
    for(int i=0; i<(int)dataset.size(); i++)
    {
        angle = dataset[i].angle;
        radius = dataset[i].range;

        if (
            angle >= -LLOBSTACLES_PROTECTION_BUBLE_BOUND
            &&
            angle <= LLOBSTACLES_PROTECTION_BUBLE_BOUND
            &&
            radius <= LLOBSTACLES_PROTECTION_BUBLE_RADIUS
        )
        {
            // protection bulle
            protection_score += 2;
        }
        else
        if (
            angle >= -LLOBSTACLES_PROTECTION_FORWARD_BOUND
            &&
            angle <= LLOBSTACLES_PROTECTION_FORWARD_BOUND
            &&
            radius <= LLOBSTACLES_PROTECTION_FORWARD_RADIUS
        )
        {
            // forward cone
            protection_score += 1;
        }
    }
}

void Dataset::clustering(void)
{
    const int min_pts = CLUSTERING_MIN_POINTS;
    const float epsilon = CLUSTERING_EPSILON;

    int clusters_count = 0;

    DBSCAN algo(min_pts, epsilon, dataset);
    algo.run();

    for( int i = 0; i < (int)dataset.size(); i++ )
    {
        dataset[i].clusterID = algo.m_points[i].clusterID - 1;
        if (clusters_count < dataset[i].clusterID) clusters_count = dataset[i].clusterID;
    }

    clusters.resize(clusters_count+1);

    int clusterId = 0;
    while(true)
    {
        vector<float> vx;
        vector<float> vy;

        bool passed = true;

        for( int i = 0; i < (int)dataset.size(); i++ )
        {
            if ( dataset[i].clusterID == clusterId )
            {
                vx.push_back(dataset[i].x);
                vy.push_back(dataset[i].y);
                passed = false;
            }
        }

        if (passed && clusterId > 0) break;

        clusters[clusterId].x = accumulate(vx.begin(), vx.end(), 0.0f) / vx.size();
        clusters[clusterId].y = accumulate(vy.begin(), vy.end(), 0.0f) / vy.size();

        clusters[clusterId].std_x = sqrtf32(
            inner_product( vx.begin(), vx.end(), vx.begin(), 0.0f ) / vx.size() - clusters[clusterId].x * clusters[clusterId].x
        );
        clusters[clusterId].std_y = sqrtf32(
            inner_product( vy.begin(), vy.end(), vy.begin(), 0.0f ) / vy.size() - clusters[clusterId].y * clusters[clusterId].y
        );

        clusterId ++;
    }
}

bool Dataset::DatasetHighLevelFilter(Point pt)
{
    float x, y;
    Cluster cl = clusters[pt.clusterID];

    x = abs(pt.x - cl.x);
    if (x > cl.std_x * HIGHLEVEL_FILTER_NB_SIGMA)
        return true;

    y = abs(pt.y - cl.y);
    if (y > cl.std_y * HIGHLEVEL_FILTER_NB_SIGMA)
        return true;

    return false;
}

void Dataset::high_level_filter(void)
{
        dataset.erase(
        remove_if(
            dataset.begin(),
            dataset.end(),
            [this](Point pt){return this->DatasetHighLevelFilter(pt);}
        ),
        dataset.end()
    );
}

void Dataset::save_to_file(int index)
{
    ofstream File1(
    		"/home/francois/Desktop/code/CDFR2023/out/p"+to_string(index)+".csv",
			ofstream::out | ofstream::trunc
    );

    for(int i=0;i<(int)dataset.size();i++)
        File1 << dataset[i].x << "\t" << dataset[i].y << "\t" << dataset[i].clusterID << "\n";

    File1.close();

    ofstream File2(
    		"/home/francois/Desktop/code/CDFR2023/out/c"+to_string(index)+".csv",
			ofstream::out | ofstream::trunc
    );

    for(int i=0;i<(int)clusters.size();i++)
        File2 << i << "\t"<< clusters[i].x << "\t" << clusters[i].y << "\t" << clusters[i].std_x << "\t" << clusters[i].std_y << "\n";

    File2.close();

}

void Dataset::send_clusters(void)
{
	LOGF_DEBUG("Clusters count : %d", clusters.size());

	char buffer[256];
	int length = 0;

	length = sprintf(buffer, "O");

	for(int i=0; i<(int)clusters.size(); i++){
		length += sprintf(&(buffer[length]),";%.3f,%.3f", clusters[i].x, clusters[i].y);
	}

	ClientUDP::send(buffer, length);
}

void Dataset::send_points(void)
{
	LOGF_DEBUG("Clusters count : %d", clusters.size());

	char buffer[16384];
	int length = 0;

	length = sprintf(buffer, "P");

	for(int i=0; i<(int)dataset.size(); i++){
		length += sprintf(&(buffer[length]),";%.2f,%.2f", dataset[i].x, dataset[i].y);
	}

	ClientUDP::send(buffer, length);
}

void Dataset::send_points_with_clusters(void)
{

	char buffer[16384];
	int length = 0;

	Position pos;
	Odometry::getCurrentPosition(&pos);

	length = sprintf(buffer, "%.2f,%.2f,%.2f", pos.x, pos.y, pos.rot);

	for(int i=0; i<(int)clusters.size(); i++){
		length += sprintf(&(buffer[length]),";%.2f,%.2f", clusters[i].x, clusters[i].y);
	}

	length += sprintf(&(buffer[length]), ";P");

	for(int i=0; i<(int)dataset.size(); i++){
		length += sprintf(&(buffer[length]),";%.2f,%.2f", dataset[i].x, dataset[i].y);
	}

	length += Terrain::printPath(&(buffer[length]));

	ClientUDP::send(buffer, length);
}





Dataset::~Dataset()
{

}


