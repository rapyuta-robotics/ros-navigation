#include <map_server/map_server_updater.h>
#include <yaml-cpp/exceptions.h>
#include <ros/ros.h>

MapServerUpdater::MapServerUpdater(const std::string& fname)
        : _fname(fname) {
    ros::NodeHandle nh;
    _map_changed_subscriber = nh.subscribe("/map_changed", 1, &MapServerUpdater::mapChangedCallback, this);

    _map_server.reset(new MapServer(fname));
    try {
        _map_server->start();
    } catch (const YAML::BadFile& ex) {
        ROS_WARN_STREAM(fname << " not found: " << ex.what());
    }

}

void MapServerUpdater::mapChangedCallback(const std_msgs::Empty& msg) {
    _map_server.reset(new MapServer(_fname));
    try {
        _map_server->start();
    } catch (const YAML::BadFile& ex) {
        ROS_FATAL_STREAM(_fname << " not found after Map Changed called: " << ex.what());
        exit(0);
    }
}
