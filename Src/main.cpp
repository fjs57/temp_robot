#include "Lidar.h"
#include "Odometry.h"
#include "Terrain.h"

#include <unistd.h>

#include "Log.h"
#include "ClientUDP.h"

bool lidar_init_ret = false;

int main(int argc, char *argv[]) {

	Logger::EnableFileOutput();
	LOG_INFO("Program Start");

	Position target;
	target.x = -0.75;
	target.y = 0.4;
	target.rot = 0;
	Terrain::setTarget(target);

	Lidar lidar;

	lidar_init_ret = lidar.initLidar();

	if(!lidar_init_ret) return 1;

	lidar_init_ret = lidar.start();

	if(!lidar_init_ret) return 2;

	sleep(1);
	Terrain::enable();

	for(int i=0; i<atoi(argv[1]); i++)
	{
		Odometry::log();
		sleep(1);
	}

	LOG_INFO("Program End");

  return 0;
}
