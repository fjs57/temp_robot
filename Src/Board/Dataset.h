/*
 * Dataset.h
 *
 *  Created on: Sep 2, 2022
 *      Author: francois
 */

#ifndef DATASET_H_
#define DATASET_H_

#include <DataTypes.h>
#include "core/common/ydlidar_datatype.h"
#include "DBSCAN.h"
#include <iostream>
#include <mutex>

#include "Positionning.h"

class Dataset
{
private :
    Positionning positionning;

    void construct_dataset(LaserScan scan);
    void low_level_filter(void);
    void referential_transform(void);

    void positionning_process(void);

    void medium_level_filter(void);
    void obstacle_detection(void);
    void clustering(void);
    void high_level_filter(void);
    bool DatasetHighLevelFilter(Point pt);

protected :
    int protection_score;
    std::vector<Point> dataset;
    std::vector<Cluster> clusters;

public :
    Dataset();

    void process(LaserScan scan);
    void positionning_start(void);

    void save_to_file(int index);
    void send_clusters(void);
    void send_points(void);
    void send_points_with_clusters(void);
    ~Dataset();
};



#endif /* DATASET_H_ */
