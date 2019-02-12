#define USAGE                                           \
    "\nUSAGE: map_server <map.yaml>\n"                  \
    "  map.yaml: map description file\n"                \
    "DEPRECATED USAGE: map_server <map> <resolution>\n" \
    "  map: image file to load\n"                       \
    "  resolution: map resolution [meters/pixel]"

#include <string>
#include <memory>

#include <map_server/map_server.h>
#include <map_server/map_server_updater.h>
#include <rr_common_config/common.h>
#include <rr_base_utils/params.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_server");
    if (argc != 2) {
        ROS_ERROR("%s", USAGE);
        exit(-1);
    }

    std::string fname(argv[1]);

    bool use_local_map;
    rapyuta::base::get_ros_param(ros::NodeHandle{"~"}, "use_local_map", use_local_map);

    try {
        if (use_local_map) {
            MapServer ms(fname);
            ros::spin();
        } else {
            MapServerUpdater ms(fname);
            ros::spin();
        }
    } catch (std::runtime_error& e) {
        ROS_ERROR("map_server exception: %s", e.what());
        return -1;
    }

    return 0;
}
