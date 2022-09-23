/*
 * Lidar.cpp
 *
 *  Created on: Sep 1, 2022
 *      Author: francois
 */

#include "Lidar.h"
#include "config.h"
#include "Log.h"

using namespace std;
using namespace ydlidar;

Lidar::Lidar()
{
	status = true;
	state = false;
	file_index = 0;
	data_mutex.unlock();
	process_state = false;
}

Lidar::~Lidar()
{
	stop();
	laser.disconnecting();
}

bool Lidar::initLidar(void)
{

	ydlidar::os_init();
	//////////////////////string property/////////////////
	/// lidar port
	std::string port = LIDAR_COMPORT;
	laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
	/// ignore array
	std::string ignore_array;
	ignore_array.clear();
	laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
					ignore_array.size());

	bool isSingleChannel = false;

	//////////////////////int property/////////////////
	/// lidar baudrate
	int baudrate = LIDAR_BAUDRATE;
	laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
	/// tof lidar
	int optval = TYPE_TRIANGLE;
	laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
	/// device type
	optval = YDLIDAR_TYPE_SERIAL;
	laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
	/// sample rate
	optval = isSingleChannel ? 3 : 4;
	laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
	/// abnormal count
	optval = 4;
	laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

	//////////////////////bool property/////////////////

	/// fixed angle resolution
	bool b_optvalue = false;

	laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));

	/// rotate 180
	laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));

	/// Counterclockwise
	laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
	b_optvalue = true;
	laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

	/// one-way communication
	laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));

	/// intensity
	b_optvalue = false;
	laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));

	/// Motor DTR
	b_optvalue = true;
	laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

	/// HeartBeat
	b_optvalue = false;
	laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

	//////////////////////float property/////////////////

	/// unit: °
	float f_optvalue = 180.0f;
	laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
	f_optvalue = -180.0f;
	laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

	/// unit: m
	f_optvalue = 64.f;
	laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
	f_optvalue = 0.05f;
	laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));

	/// unit: Hz
	float frequency = 9.f;
	laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

	return laser.initialize();

}

bool Lidar::start(void)
{
	status = laser.turnOn();
	startThread();
	computePosition();
	return status;
}

void Lidar::stop(void)
{
	stopThread();
	laser.turnOff();
}

void Lidar::startThread(void)
{
	process_state = true;
	process_thread = std::thread([this]{ this->process(); });
}

void Lidar::stopThread(void)
{
	process_state = false;
	process_thread.join();
}

void Lidar::process(void)
{
	while(process_state)
	{
		if(!ydlidar::os_isOk()) break;

		if (laser.doProcessSimple(scan)) {
			dataProcess();
		}
	}
}

void Lidar::dataProcess(void)
{
	data_mutex.lock();
	dataset.process(scan);
	data_mutex.unlock();
	//dataset.save_to_file(file_index++);
}

void Lidar::computePosition(void)
{
	dataset.positionning_start();
}


