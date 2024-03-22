#include "inspire_hand.hpp"
#include "inspire_hand/setAngleSrv.h"
#include <ros/ros.h>

bool setAngleCallback(inspire_hand::setAngleSrv::Request &req, inspire_hand::setAngleSrv::Response &res,
                      inspire_hand::InspireHandSerial *inspire_hand_serial)
{
    inspire_hand_serial->setAngle(req.id, {req.angle0, req.angle1, req.angle2, req.angle3, req.angle4, req.angle5});
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inspire_hand_server");
    string port;
    int baudrate;
    ros::NodeHandle nh;
    string node_name = ros::this_node::getName();
    cout << node_name << endl;
    if (!nh.getParam(node_name + "/port", port))
    {
        ROS_ERROR("Failed to get param 'port'");
        return -1;
    }
    if (!nh.getParam(node_name + "/baudrate", baudrate))
    {
        ROS_ERROR("Failed to get param 'baudrate'");
        return -1;
    }
    inspire_hand::InspireHandSerial inspire_hand_serial(port, baudrate);
    ros::ServiceServer service = nh.advertiseService<inspire_hand::setAngleSrv::Request, inspire_hand::setAngleSrv::Response>(node_name + "/set_angle", boost::bind(setAngleCallback, _1, _2, &inspire_hand_serial));
    ros::spin();
    return 0;
}
