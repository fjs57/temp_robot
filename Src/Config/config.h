/*
 * config.h
 *
 *  Created on: Sep 1, 2022
 *      Author: francois
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// LOGGING ------------------------------------------------------------
#define LOG_FILE_ENABLE false
#define LOG_DEFAULT_FILEPATH "Logs/cdfr.log"
// DebugPriority, TracePriority, InfoPriority, WarnPriority, ErrorPriority, CriticalPriority
#define LOG_DEFAULT_LEVEL TracePriority

// LIDAR ------------------------------------------------------------
// #define LIDAR_COMPORT "/dev/ydlidar"
#include "lidar_comport.h"
#define LIDAR_BAUDRATE 128000

#define LIDAR_DETECTION_RADIUS 4.f
#define LIDAR_MINIMUM_DETECTION_RADIUS 0.1f

// FIELD SIZE/PROPS ------------------------------------------------------------
#define TERRAIN_SIZE_X 1.8f
#define TERRAIN_SIZE_Y 1.2f

#define FIELD_BOUND_X_MIN -TERRAIN_SIZE_X / 2
#define FIELD_BOUND_X_MAX  TERRAIN_SIZE_X / 2
#define FIELD_BOUND_Y_MIN -TERRAIN_SIZE_Y / 2
#define FIELD_BOUND_Y_MAX  TERRAIN_SIZE_Y / 2

#define TERRAIN_FILTER_MARGIN 0.15f

// LIDAR DETECTION AND PROTECTION ------------------------------------------------------------

#define LLOBSTACLES_PROTECTION_BUBLE_RADIUS 0.2f // 30cm
#define LLOBSTACLES_PROTECTION_BUBLE_BOUND 1.571f // pi/2 ==> 90°

#define LLOBSTACLES_PROTECTION_FORWARD_RADIUS 0.5f // 50cm
#define LLOBSTACLES_PROTECTION_FORWARD_BOUND 0.524f // pi/6 ==> 30°

#define LLOBSTACLES_PROTECTION_MAX_SCORE 5

#define HIGHLEVEL_FILTER_NB_SIGMA 3

// CLUSTERING ------------------------------------------------------------

#define CLUSTERING_MIN_POINTS 3
#define CLUSTERING_EPSILON 0.01f

// POSITIONNING ------------------------------------------------------------

#define POSITIONNING_CONTINUOUS_MEASURE

#define POSITIONNING_MAX_STDDEV 0.15f
#define POSITIONNING_SAMPLING_REPETITIONS 3
#define POSITIONNING_MAX_ERROR 0.05f

#define BEACON_POSITION_A_X -0.93f
#define BEACON_POSITION_A_Y  0.57f

#define BEACON_POSITION_B_X -0.93f
#define BEACON_POSITION_B_Y -0.57f

#define BEACON_POSITION_C_X  0.93f
#define BEACON_POSITION_C_Y  0.f

#define POSITIONNING_TRACE

// ODOMETRY ---------------------------------------------------------------

//PositionSourceOdometry, PositionSourceLidar
#define ODOMETRY_DEFAULT_POSITION_SOURCE PositionSourceLidar

// PATH PLANNER -----------------------------------------------------------
#define PATH_PLANNER_ROBOT_RADIUS 0.15f
#define PATH_PLANNER_OBSTACLES_RADIUS 0.05f

#define PATH_PLANNER_PROTECTION_AROUND_OBSTACLES PATH_PLANNER_ROBOT_RADIUS+PATH_PLANNER_OBSTACLES_RADIUS

#define PATH_PLANNER_GRID_MESH_SIZE 0.02f

// UDP SERVER -----------------------------------------------------------
#define UDP_ADDR "224.0.0.80"
#define UDP_PORT 4003

#endif /* CONFIG_H_ */
