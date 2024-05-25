#!/bin/bash
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="serial"
catkin_make -DCATKIN_WHITELIST_PACKAGES="RMUC_msgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_driver"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

