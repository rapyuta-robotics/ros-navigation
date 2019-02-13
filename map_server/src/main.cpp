#define USAGE                                           \
    "\nUSAGE: map_server <map.yaml>\n"                  \
    "  map.yaml: map description file\n"

#include <string>
#include <memory>

#include <map_server/map_server.h>
#include <map_server/map_server_updater.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_server");
    if (argc != 2 && argc != 3) {
        ROS_ERROR("%s", USAGE);
        exit(-1);
    }

    if (argc != 2) {
        ROS_WARN("Using deprecated map server interface. Please switch to new interface.");
    }

    std::string fname(argv[1]);
    double res = (argc == 2) ? 0.0 : atof(argv[2]);

    bool use_local_map = true;
    ros::NodeHandle nh{"~"};
    bool has_param = nh.getParam("use_local_map", use_local_map);
    if (!has_param) {
        use_local_map = true;
    }

    try {
        if (use_local_map) {
            if (argc == 2) {
                MapServer ms(fname);
                ros::spin();
            } else {
                MapServer ms(fname, res);
                ros::spin();
            }

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
