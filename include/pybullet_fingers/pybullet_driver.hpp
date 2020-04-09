/**
 * @file
 * @brief C++ wrappers to use pyBullet simulation of fingers as
 * robot_interfaces::RobotDriver.
 */
#pragma once

#include <chrono>
#include <thread>

#include <pybind11/eigen.h>
#include <pybind11/embed.h>

#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/trifinger_types.hpp>

namespace pybullet_fingers
{
namespace py = pybind11;

/**
 * @brief Base driver for pyBullet of both single Finger and TriFinger.
 *
 * Implements all methods of RobotDriver except `initialize` which needs to be
 * implemented by the child class as there are differences between single Finger
 * and TriFinger.
 *
 * All other methods are generic and only need to be templated with the proper
 * types for actions/observations.
 *
 * @tparam Vector  The vector type used for the specific robot.
 * @tparam Action  Action type used for the specific robot.
 * @tparam Observation  Observation type used for the specific robot.
 */
template <typename Vector, typename Action, typename Observation>
class BasePyBulletFingerDriver
    : public robot_interfaces::RobotDriver<Action, Observation>
{
protected:
    //! @brief If true, step simulation at 1 kHz, otherwise as fast as possible
    bool real_time_mode_;

    //! @brief If true, pyBullet GUI for visualization is started.
    bool visualize_;

    /**
     * @brief Instance of the Python class SimFinger that implements the
     *        pyBullet simulation of the finger robots.
     *
     * This needs to be initialized by the child class!
     */
    py::object sim_finger_;

public:
    BasePyBulletFingerDriver(bool real_time_mode, bool visualize)
        : real_time_mode_(real_time_mode), visualize_(visualize)
    {
    }

    Observation get_latest_observation() override
    {
        Observation observation;

        py::gil_scoped_acquire acquire;

        // get latest observation
        py::object py_obs = sim_finger_.attr("_get_latest_observation")();

        // TODO this would be even simpler when SimFinger would use the correct
        // observation type as there are already bindings for it
        observation.position = py_obs.attr("position").cast<Vector>();
        observation.velocity = py_obs.attr("velocity").cast<Vector>();
        observation.torque = py_obs.attr("torque").cast<Vector>();

        return observation;
    }

    Action apply_action(const Action &desired_action) override
    {
        auto start_time = std::chrono::system_clock::now();

        Action applied_action;
        {
            // wrap python calls in a block so GIL is only kept as long as it
            // is really needed
            py::gil_scoped_acquire acquire;

            py::object py_action = py::cast(desired_action);
            py::object py_applied_action =
                sim_finger_.attr("_set_desired_action")(py_action);
            sim_finger_.attr("_step_simulation")();

            applied_action = py_applied_action.cast<Action>();
        }

        if (real_time_mode_)
        {
            using namespace std::chrono_literals;
            std::this_thread::sleep_until(start_time + 1ms);
        }

        return applied_action;
    }

    std::string get_error() override
    {
        return "";  // no errors
    }

    void shutdown() override
    {
        py::gil_scoped_acquire acquire;
        sim_finger_.attr("disconnect_from_simulation")();
    }
};

/**
 * @brief Create a Finger/TriFinger-backend using pyBullet.
 *
 * @tparam Types  The struct providing the types for action, observation, etc.
 * @tparam Driver  pyBullet-Driver class for either single Finger or TriFinger.
 *
 * @param robot_data RobotData instance for the backend.
 * @param real_time_mode  If true, step the simulation in real time, otherwise
 *     as fast as possible.
 * @param visualize If true, pyBullet's GUI is started for visualization.
 *
 * @return Backend using a driver of the specified type.
 */
template <typename Types, typename Driver>
typename Types::BackendPtr create_finger_backend(
    typename Types::BaseDataPtr robot_data, bool real_time_mode, bool visualize)
{
    auto robot = std::make_shared<Driver>(real_time_mode, visualize);
    auto backend = std::make_shared<typename Types::Backend>(
        robot, robot_data, real_time_mode);

    if (real_time_mode)
    {
        // in real-time mode use action repetition like on the real robot
        backend->set_max_action_repetitions(
            std::numeric_limits<uint32_t>::max());
    }
    else
    {
        // in non-real-time mode, disable repetitions as the simulation can
        // simply wait
        backend->set_max_action_repetitions(0);
    }

    return backend;
}

/**
 * @brief pyBullet driver for the single Finger.
 */
class PyBulletSingleFingerDriver
    : public BasePyBulletFingerDriver<
          robot_interfaces::FingerTypes::Vector,
          robot_interfaces::FingerTypes::Action,
          robot_interfaces::FingerTypes::Observation>
{
public:
    typedef robot_interfaces::FingerTypes::Vector Vector;

    // inherit the constructor
    using BasePyBulletFingerDriver::BasePyBulletFingerDriver;

    void initialize() override
    {
        py::gil_scoped_acquire acquire;

        py::module sim_finger =
            py::module::import("pybullet_fingers.sim_finger");
        sim_finger_ = sim_finger.attr("SimFinger")(0.001, visualize_, "single");

        Vector initial_position;
        initial_position << 0, -0.7, -1.5;
        sim_finger_.attr("reset_finger")(initial_position);
    }
};

/**
 * @brief pyBullet driver for the TriFinger
 */
class PyBulletTriFingerDriver
    : public BasePyBulletFingerDriver<
          robot_interfaces::TriFingerTypes::Vector,
          robot_interfaces::TriFingerTypes::Action,
          robot_interfaces::TriFingerTypes::Observation>
{
public:
    typedef robot_interfaces::TriFingerTypes::Vector Vector;

    // inherit the constructor
    using BasePyBulletFingerDriver::BasePyBulletFingerDriver;

    void initialize() override
    {
        py::gil_scoped_acquire acquire;

        py::module sim_finger =
            py::module::import("pybullet_fingers.sim_finger");
        sim_finger_ = sim_finger.attr("SimFinger")(0.001, visualize_, "tri");

        Vector initial_position;
        initial_position << 0, -0.7, -1.5, 0, -0.7, -1.5, 0, -0.7, -1.5;
        sim_finger_.attr("reset_finger")(initial_position);
    }
};

}  // namespace pybullet_fingers
