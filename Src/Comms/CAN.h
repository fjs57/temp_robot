#include <stdint.h>
#include <thread>
#include <mutex>
#include "Log.h"
#include <linux/can.h>

class CAN {
    public:

        static void init(void){
            getInstance();
        }

        static int send(uint16_t address, uint8_t *data, uint8_t length){
            return getInstance()._send(address, data, length);
        }

        static void log(struct can_frame frame, bool rx_tx){
            char buf[128];
            int len, i;

            if(rx_tx)
                len = sprintf(buf, "CAN Msg received addr=%03X dlc=%d", frame.can_id, frame.can_dlc);
            else
                len = sprintf(buf, "CAN Msg sent addr=%03X dlc=%d", frame.can_id, frame.can_dlc);

            if(frame.can_dlc>0){

                len += sprintf(&(buf[len]), " data=[%02X", frame.data[0]);

                for(i=1;i<frame.can_dlc;i++){
                    len += sprintf(&(buf[len]), ",%02X", frame.data[i]);
                }

                len += sprintf(&(buf[len]), "]");
            }

            LOGF_TRACE("%s", buf);
        }

    private:

        std::thread process_thread;
        std::mutex send_mutex;
        int initialized;
        bool process_state;
        int can_socket;

        static CAN& getInstance(void){
            static CAN instance;
            return instance;
        }

        void _process(void);
        int _init(void);
        int _send(uint16_t address, uint8_t *data, uint8_t length);
        void _startThread(void);
        void _stopThread(void);
        void _onReceive(struct can_frame frame);

        CAN();
        ~CAN();
};