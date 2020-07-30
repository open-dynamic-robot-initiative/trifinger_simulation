#!/bin/bash
# Update the local robot_properties_fingers files

trifinger_simulation=$(rospack find trifinger_simulation)
robot_properties_fingers=$(rospack find robot_properties_fingers)
local_properties=${trifinger_simulation}/python/trifinger_simulation/robot_properties_fingers

rm -rf ${local_properties}/*
cp -r ${robot_properties_fingers}/meshes ${local_properties}
cp -r ${robot_properties_fingers}/urdf ${local_properties}
