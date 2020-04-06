cmake_minimum_required(VERSION 2.8.3)
project(pybullet_fingers)

find_package(catkin REQUIRED COMPONENTS
    mpi_cmake_modules
    pybind11_catkin
    robot_interfaces
)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)


# Python Bindings
pybind11_add_module(drivers srcpy/drivers.cpp)
target_link_libraries(drivers PRIVATE ${catkin_LIBRARIES})
set_target_properties(drivers PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(TARGETS drivers DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})


# build with -DBUILD_DOCUMENTATION=ON to build documentation
build_doxygen_documentation()


# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
