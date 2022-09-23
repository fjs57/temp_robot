/*
 * ClientUDP.h
 *
 *  Created on: Sep 6, 2022
 *      Author: francois
 */

#ifndef CLIENTUDP_H_
#define CLIENTUDP_H_

#include "UDP.h"
#include <mutex>
#include "config.h"

class ClientUDP {

private :

	udp_client_server::udp_client udp;
	std::mutex client_mutex;

	ClientUDP():udp(UDP_ADDR, UDP_PORT){}

	~ClientUDP(){
		client_mutex.unlock();
	}

	int _send(const char *msg, size_t size){
		client_mutex.lock();
		int ret = udp.send(msg, size);
		client_mutex.unlock();
		return ret;
	}

public :

	static ClientUDP& getInstance(void){
		static ClientUDP server;
		return server;
	}

	static int send(const char *msg, size_t size){
		 return getInstance()._send(msg, size);
	}

	template<typename ... Args>
	static int sendDatagram(const char *msg, Args... args){
		char buffer[256];
		int length = sprintf(buffer, msg, args...);
		return send(buffer, length);
	}

};

#endif /* CLIENTUDP_H_ */
