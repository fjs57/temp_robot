/*
 * LidarDataTypes.h
 *
 *  Created on: Sep 2, 2022
 *      Author: francois
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

typedef struct Point_
{
    float angle, range;
    float x, y, z;  // X, Y, Z position
    int clusterID;  // clustered ID
}Point;

typedef struct Cluster_ {
    float x;
    float y;
    float std_x;
    float std_y;
    float max_x;
    float min_x;
    float max_y;
    float min_y;
    int count;
    bool isBeacon;
    int id;
} Cluster;

typedef struct Position_ {
	float x;
	float y;
	float rot;
} Position;

enum PositionSource{
	PositionSourceOdometry,
	PositionSourceLidar
};


#endif /* DATATYPES_H_ */
