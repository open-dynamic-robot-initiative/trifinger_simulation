#!/bin/bash
# Update the local robot_properties_fingers files

pybullet_fingers=$(rospack find pybullet_fingers)
robot_properties_fingers=$(rospack find robot_properties_fingers)
local_properties=${pybullet_fingers}/python/pybullet_fingers/robot_properties_fingers

rm -rf ${local_properties}/*
cp -r ${robot_properties_fingers}/meshes ${local_properties}
cp -r ${robot_properties_fingers}/urdf ${local_properties}
