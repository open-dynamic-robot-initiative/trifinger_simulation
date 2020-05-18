#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include <pybullet_fingers/pybullet_driver.hpp>
#include <robot_interfaces/finger_types.hpp>

using namespace pybind11::literals;

PYBIND11_MODULE(drivers, m)
{
    m.def("create_single_finger_backend",
          &pybullet_fingers::create_finger_backend<
              robot_interfaces::MonoFingerTypes,
              pybullet_fingers::PyBulletSingleFingerDriver>,
          "robot_data"_a,
          "real_time_mode"_a,
          "visualize"_a,
          "first_action_timeout"_a = std::numeric_limits<double>::infinity(),
          R"XXX(
            Create backend for the Single Finger robot using pyBullet simulation.

            Args:
                robot_data (robot_interfaces.finger.Data):  Robot data instance
                    for the Finger robot.
                real_time_mode (bool):  If True, step the simulation in real
                    time, otherwise as fast as possible.
                visualize (bool):  If True, the pyBullet GUI is started for
                    visualization.

            Returns:
                Finger backend using simulation instead of the real robot.
)XXX");

    m.def("create_trifinger_backend",
          &pybullet_fingers::create_finger_backend<
              robot_interfaces::TriFingerTypes,
              pybullet_fingers::PyBulletTriFingerDriver>,
          "robot_data"_a,
          "real_time_mode"_a,
          "visualize"_a,
          "first_action_timeout"_a = std::numeric_limits<double>::infinity(),
          R"XXX(
            Create a backend for the TriFinger robot using pyBullet simulation.

            Args:
                robot_data (robot_interfaces.trifinger.Data):  Robot data instance
                    for the TriFinger robot.
                real_time_mode (bool):  If True, step the simulation in real
                    time, otherwise as fast as possible.
                visualize (bool):  If True, the pyBullet GUI is started for
                    visualization.

            Returns:
                TriFinger backend using simulation instead of the real robot.
)XXX");
}
