/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

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
    } catch (const std::runtime_error& e) {
        ROS_ERROR("map_server exception: %s", e.what());
        return -1;
    }

    return 0;
}
