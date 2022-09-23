#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can/raw.h>

#include "config.h"
#include "CAN.h"

CAN::CAN(){
    initialized = -1;
    if(_init()!=0){
        LOG_CRITICAL("CAN Init Failed");
        exit(1);
    } else {
        initialized = 1;
    }
}

CAN::~CAN(){
    process_state = false;
    if(process_thread.joinable()) process_thread.join();
    send_mutex.unlock();
}

int CAN::_init(void){

    int ret;
    struct ifreq ifr;
    struct sockaddr_can addr;    
    struct can_filter rfilter[1];

    LOG_INFO("CAN Init start");

    // close the IF in case of badly closed
    system("sudo ifconfig can0 down");

    // set the parameters of the IF
    ret = system(CAN_BITRATE_CMD);
    if(ret!=0){
        LOG_ERROR("Cannot set the CAN Interface parameters");
    }

    // set the IF state as UP
    ret = system("sudo ifconfig can0 up");
    if(ret!=0){
        LOG_CRITICAL("Cannot turn on the CAN interface");
        return 1;
    }

    // create the socket
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        LOG_CRITICAL("Cannot init CAN : socket PF_CAN failed");
        return 1;
    }

    // specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(can_socket, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        LOG_CRITICAL("Cannot init CAN : ioctl failed");
        return 1;
    }

    // bind the socket to can0
    addr.can_family = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(can_socket, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        LOG_CRITICAL("Cannot init CAN : bind failed");
        return 1;
    }


    rfilter[0].can_id = CAN_RX_FILTER_ID;
    rfilter[0].can_mask = CAN_RX_FILTER_MASK;
    ret = setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    if(ret!=0){
        LOG_CRITICAL("Cannot init CAN : rx filter cannot be set");
        return 1;
    }

    initialized = 1;
    LOG_INFO("CAN interface initialized sucessfully");

    _startThread();

    return 0;
}

void CAN::_stopThread(void){
    process_state = false;
}

void CAN::_startThread(void){
    process_state = true;
    process_thread = std::thread([this]{ this->_process(); });
}

int CAN::_send(uint16_t address, uint8_t *data, uint8_t length){

    if(initialized==-1){
        LOG_ERROR("Sending Failed. Trying to send before initialized");
        return 1;
    }

    int i, nbytes;
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));

    frame.can_id = address;
    frame.can_dlc = length;
    for(i=0;i<length;i++) frame.data[i] = data[i];

    log(frame, false);

    nbytes = write(can_socket, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        LOG_ERROR("Send Error !");
        return _init();
    }
    LOG_TRACE("Send success");
    return 0;
}

void CAN::_process(void){
    int nbytes;
    struct can_frame frame;

    while(process_state){

        if(initialized==-1){
            continue;
        }

        /* Read the interface and LL decode */
            
        memset(&frame, 0, sizeof(struct can_frame));
        nbytes = read(can_socket, &frame, sizeof(frame));
        if(nbytes > 0) {
            _onReceive(frame);
        }
    }
}

void CAN::_onReceive(struct can_frame frame){
    log(frame, true);
}
