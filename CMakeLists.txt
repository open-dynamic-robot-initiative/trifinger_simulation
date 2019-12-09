cmake_minimum_required(VERSION 2.8.3)
project(pybullet_fingers)

find_package(catkin REQUIRED COMPONENTS
  mpi_cmake_modules
)

catkin_python_setup()

catkin_package()

# build with -DBUILD_DOCUMENTATION=ON to build documentation
build_doxygen_documentation()
