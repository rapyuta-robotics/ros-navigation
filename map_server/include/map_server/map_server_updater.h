#pragma once

#include <string>
#include <memory>
#include <map_server/map_server.h>
#include <std_msgs/Empty.h>

class MapServerUpdater {
public:
    MapServerUpdater(const std::string& fname);

private:
    std::unique_ptr<MapServer> _map_server;
    std::string _fname;

    ros::Subscriber _map_changed_subscriber;

    void mapChangedCallback(const std_msgs::Empty& msg);
};