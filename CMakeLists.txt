cmake_minimum_required(VERSION 3.1)
project(pybullet_fingers)

# Specify C++ Standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)


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
catkin_add_pybind11_module(drivers)


# build with -DBUILD_DOCUMENTATION=ON to build documentation
build_doxygen_documentation()


# Tests
if (CATKIN_ENABLE_TESTING)
    # These tests require Python 3 but catkin will execute them with Python 2,
    # therefore disable the automatic execution (they can still be executed
    # manually to verify that they pass).
    #catkin_add_nosetests(test/test_determinism.py)
    #catkin_add_nosetests(test/test_pybullet_backend.py)


    macro(create_unittest test_name)
        # create the executable
        catkin_add_gtest(${test_name}
          tests/${test_name}.cpp
        )
        if(TARGET ${test_name})
            # link the dependencies to it
            target_link_libraries(${test_name}
                ${catkin_LIBRARIES}
            )
        endif()
    endmacro(create_unittest test_name)

    create_unittest(test_pybullet_backend)

endif()
