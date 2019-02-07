#pragma once

#include <string>
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>

class MapServer {
public:
    MapServer(const std::string& fname);

private:
    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

    ros::NodeHandle _nh;
    ros::Publisher _map_pub;
    ros::Publisher _metadata_pub;
    ros::ServiceServer _service;

    nav_msgs::MapMetaData _meta_data_message;
    nav_msgs::GetMap::Response _map_resp;
};