# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "base_local_planner;nav_core;pluginlib;roscpp;tf;tf2_ros;costmap_2d;nav_msgs;dynamic_reconfigure;angles".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lpid_local_planner".split(';') if "-lpid_local_planner" != "" else []
PROJECT_NAME = "pid_local_planner"
PROJECT_SPACE_DIR = "/home/agrobot/Robot_ws/install"
PROJECT_VERSION = "0.0.0"
