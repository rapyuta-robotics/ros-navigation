#include <map_server/map_server.h>
#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>
#include <libgen.h>

MapServer::MapServer(const std::string& fname)
        : _fname(fname)
        , _res(0) {}

MapServer::MapServer(const std::string& fname, const double res)
        : _fname(fname)
        , _res(res) {}

void MapServer::start() {
    if (_res != 0.0) {
        start_deprecated();
    }

    std::string mapfname;

    float res;
    float negate;

    double origin[3];
    double occ_th, free_th;

    MapMode mode;
    std::string frame_id;
    ros::NodeHandle private_nh("~");
    private_nh.param("frame_id", frame_id, std::string("map"));

    // Load map file information
    YAML::Node doc = YAML::LoadFile(_fname);

    try {
        res = doc["resolution"].as<float>();
    } catch (const YAML::InvalidScalar&) {
        ROS_FATAL_STREAM("The map does not contain a resolution tag or it is invalid.");
        exit(0);
    }

    try {
        negate = doc["negate"].as<float>();
    } catch (const YAML::InvalidScalar&) {
        ROS_FATAL_STREAM("The map does not contain a negate tag or it is invalid.");
        exit(0);
    }

    try {
        occ_th = doc["occupied_thresh"].as<double>();
    } catch (const YAML::InvalidScalar&) {
        ROS_FATAL_STREAM("The map does not contain a occupied_thresh tag or it is invalid.");
        exit(0);
    }

    try {
        free_th = doc["free_thresh"].as<double>();
    } catch (const YAML::InvalidScalar&) {
        ROS_FATAL_STREAM("The map does not contain a free_thresh tag or it is invalid.");
        exit(0);
    }

    try {
        std::string mode_str = doc["mode"].as<std::string>();

        if (mode_str == "trinary")
            mode = MapMode::TRINARY;
        else if (mode_str == "scale")
            mode = MapMode::SCALE;
        else if (mode_str == "raw")
            mode = MapMode::RAW;
        else {
            ROS_FATAL("Invalid mode tag \"%s\".", mode_str.c_str());
            exit(0);
        }

    } catch (const YAML::Exception&) {
        ROS_DEBUG_STREAM("The map does not contain a mode tag or it is invalid... assuming Trinary");
        mode = MapMode::TRINARY;
    }

    try {
        origin[0] = doc["origin"][0].as<double>();
        origin[1] = doc["origin"][1].as<double>();
        origin[2] = doc["origin"][2].as<double>();
    } catch (const YAML::InvalidScalar&) {
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        exit(-1);
    }

    try {
        mapfname = doc["image"].as<std::string>();

        if (mapfname.size() == 0) {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
        }

        if (mapfname[0] != '/') {
            // dirname can modify what you pass it
            char* fname_copy = strdup(_fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
        }

    } catch (const YAML::InvalidScalar&) {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        exit(-1);
    }

    ROS_INFO_STREAM("Loading map from image: " << mapfname);

    try {
        map_server::loadMapFromFile(&_map_resp, mapfname.c_str(), res, negate, occ_th, free_th, origin, mode);
    } catch (std::runtime_error e) {
        ROS_ERROR("%s", e.what());
        exit(-1);
    }

    // To make sure get a consistent time in simulation
    ros::Time::waitForValid();

    _map_resp.map.info.map_load_time = ros::Time::now();
    _map_resp.map.header.frame_id = frame_id;
    _map_resp.map.header.stamp = ros::Time::now();

    ROS_INFO("Read a %d X %d map @ %.3lf m/cell", _map_resp.map.info.width, _map_resp.map.info.height,
            _map_resp.map.info.resolution);

    _meta_data_message = _map_resp.map.info;

    _service = _nh.advertiseService("static_map", &MapServer::mapCallback, this);

    // Latched publisher for metadata
    _metadata_pub = _nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    _metadata_pub.publish(_meta_data_message);

    // Latched publisher for data
    _map_pub = _nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    _map_pub.publish(_map_resp.map);
}

void MapServer::start_deprecated() {
    std::string mapfname = _fname;

    float negate = 0;

    double origin[3] = {0, 0, 0};
    double occ_th = 0.65, free_th = 0.196;

    MapMode mode;
    std::string frame_id;

    ros::NodeHandle private_nh("~");
    private_nh.getParam("negate", negate);
    private_nh.getParam("occupied_thresh", occ_th);
    private_nh.getParam("free_thresh", free_th);

    ROS_INFO_STREAM("Loading map from image: " << mapfname);

    try {
        map_server::loadMapFromFile(&_map_resp, mapfname.c_str(), _res, negate, occ_th, free_th, origin, mode);
    } catch (std::runtime_error e) {
        ROS_ERROR("%s", e.what());
        exit(-1);
    }

    // To make sure get a consistent time in simulation
    ros::Time::waitForValid();

    _map_resp.map.info.map_load_time = ros::Time::now();
    _map_resp.map.header.frame_id = frame_id;
    _map_resp.map.header.stamp = ros::Time::now();

    ROS_INFO("Read a %d X %d map @ %.3lf m/cell", _map_resp.map.info.width, _map_resp.map.info.height,
            _map_resp.map.info.resolution);

    _meta_data_message = _map_resp.map.info;

    _service = _nh.advertiseService("static_map", &MapServer::mapCallback, this);

    // Latched publisher for metadata
    _metadata_pub = _nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    _metadata_pub.publish(_meta_data_message);

    // Latched publisher for data
    _map_pub = _nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    _map_pub.publish(_map_resp.map);
}

bool MapServer::mapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res) {
    res = _map_resp;
    ROS_INFO("Sending map");
    return true;
}