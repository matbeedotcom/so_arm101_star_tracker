#include <iostream>
#include "SCServo.h"
#include <thread>
#include <chrono>

SMS_STS sm_st;

int main()
{
    const char* port = "/dev/ttyACM0";
    std::cout << "Scanning for servos on " << port << std::endl;

    if(!sm_st.begin(1000000, port)){
        std::cout << "Failed to init serial port!" << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Scan servo IDs 1-20
    for(int id = 1; id <= 20; id++){
        std::cout << "Pinging servo ID " << id << "... ";
        int response = sm_st.Ping(id);
        if(response != -1){
            std::cout << "FOUND! Response: " << response << std::endl;
        } else {
            std::cout << "no response" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    sm_st.end();
    return 0;
}