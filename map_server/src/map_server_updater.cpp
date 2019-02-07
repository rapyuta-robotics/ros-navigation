#include <map_server/map_server_updater.h>
#include <ros/ros.h>

MapServerUpdater::MapServerUpdater(const std::string& _fname)
        : _fname(_fname) {
    ros::NodeHandle nh;
    _map_changed_subscriber = nh.subscribe("/map_changed", 1, &MapServerUpdater::mapChangedCallback, this);
}

void MapServerUpdater::mapChangedCallback(const std_msgs::Empty& msg) {
    _map_server.reset(new MapServer(_fname));
}
