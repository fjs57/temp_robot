#include <stdint.h>
#include <iostream>
#include <thread>

class CAN {
    public:

        static void send(uint16_t address, uint8_t *data, uint8_t length){
            std::cout << "Hi from send function !" << std::endl;
        }

    private:

        std::thread process_thread;

        static CAN& getInstance(void){
            static CAN instance;
            return instance;
        }

        void process(void){

        }
        CAN():process_thread(&process){

        }
        ~CAN(){

        }




};