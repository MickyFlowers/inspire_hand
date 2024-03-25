#include "inspire_hand.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inspire_hand_test");
    inspire_hand::InspireHandSerial inspire_hand_serial("/dev/ttyUSB0", 115200);
    inspire_hand_serial.clearError(1);
    
}
