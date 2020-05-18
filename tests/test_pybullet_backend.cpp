/**
 * @file
 * @copyright Copyright (c) 2020, Max Planck Gesellschaft.
 */
#include <gtest/gtest.h>

#include <memory>

#include <pybullet_fingers/pybullet_driver.hpp>
#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/robot_frontend.hpp>

constexpr bool VISUALIZE = false;
constexpr double POSITION_TOLERANCE = 0.05;

// Simple test where the robot is controlled to reach a list of positions and
// after each step it is checked whether that position was actually reached.

TEST(TestPyBulletDriver, monofingerone)
{
    auto robot_data = std::make_shared<
        robot_interfaces::MonoFingerTypes::SingleProcessData>();

    auto backend = pybullet_fingers::create_finger_backend<
        robot_interfaces::MonoFingerTypes,
        pybullet_fingers::PyBulletSingleFingerDriver>(
        robot_data, VISUALIZE, VISUALIZE);

    auto frontend = robot_interfaces::MonoFingerTypes::Frontend(robot_data);

    backend->initialize();

    // Need to release the GIL in the main thread, otherwise the driver
    // (running in the backend thread) is blocked.
    pybind11::gil_scoped_release foo;

    typedef robot_interfaces::MonoFingerTypes::Action Action;

    std::array<Action::Vector, 3> goals;
    goals[0] << 0.21, 0.32, -1.10;
    goals[1] << 0.69, 0.78, -1.07;
    goals[2] << -0.31, 0.24, -0.20;

    for (Action::Vector goal : goals)
    {
        unsigned int t;
        auto action = Action::Position(goal);
        for (int i = 0; i < 500; i++)
        {
            t = frontend.append_desired_action(action);
            frontend.wait_until_timeindex(t);
        }

        // check if desired position is reached
        auto actual_position = frontend.get_observation(t).position;
        // std::cout << actual_position.transpose() << std::endl;
        ASSERT_TRUE(actual_position.isApprox(goal, POSITION_TOLERANCE));
    }
}

TEST(TestPyBulletDriver, trifinger)
{
    auto robot_data =
        std::make_shared<robot_interfaces::TriFingerTypes::SingleProcessData>();

    auto backend = pybullet_fingers::create_finger_backend<
        robot_interfaces::TriFingerTypes,
        pybullet_fingers::PyBulletTriFingerDriver>(
        robot_data, VISUALIZE, VISUALIZE);

    auto frontend = robot_interfaces::TriFingerTypes::Frontend(robot_data);

    backend->initialize();

    // Need to release the GIL in the main thread, otherwise the driver
    // (running in the backend thread) is blocked.
    pybind11::gil_scoped_release foo;

    typedef robot_interfaces::TriFingerTypes::Action Action;

    std::array<Action::Vector, 3> goals;
    goals[0] << 0.27, -0.72, -1.03, 0.53, -0.26, -1.67, -0.10, 0.10, -1.36;
    goals[1] << 0.38, -0.28, -1.91, 0.22, 0.02, -0.03, 0.00, 0.28, -0.03;
    goals[2] << 0.14, 0.27, -0.03, 0.08, -0.08, -0.03, 0.53, -0.00, -0.96;

    for (Action::Vector goal : goals)
    {
        unsigned int t;
        auto action = Action::Position(goal);
        for (int i = 0; i < 500; i++)
        {
            t = frontend.append_desired_action(action);
            frontend.wait_until_timeindex(t);
        }

        // check if desired position is reached
        auto actual_position = frontend.get_observation(t).position;
        // std::cout << actual_position.transpose() << std::endl;
        ASSERT_TRUE(actual_position.isApprox(goal, POSITION_TOLERANCE));
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
