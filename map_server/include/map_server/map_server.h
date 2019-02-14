#pragma once

#include <string>
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>

class MapServer {
public:
    explicit MapServer(const std::string& fname);
    explicit MapServer(const std::string& fname, const double res);

private:
    void start();

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

    ros::NodeHandle _nh;
    ros::Publisher _map_pub;
    ros::Publisher _metadata_pub;
    ros::ServiceServer _service;

    std::string _fname;
    std::string _res;

    nav_msgs::MapMetaData _meta_data_message;
    nav_msgs::GetMap::Response _map_resp;
};