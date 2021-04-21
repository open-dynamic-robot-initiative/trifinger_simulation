import os
from setuptools import setup

package_name = "trifinger_simulation"


def find_package_data(base_dir, data_dir):
    """Get list of all files in base_dir/data_dir, relative to base_dir."""
    paths = []
    for (path, directories, filenames) in os.walk(
        os.path.join(base_dir, data_dir)
    ):
        for filename in filenames:
            paths.append(
                os.path.relpath(os.path.join(path, filename), base_dir)
            )
    return paths


setup(
    name=package_name,
    version="1.1.0",
    packages=[
        package_name,
        package_name + ".gym_wrapper",
        package_name + ".gym_wrapper.envs",
        package_name + ".tasks",
        package_name + ".tasks.move_cube",
        package_name + ".tasks.move_cube_on_trajectory",
    ],
    package_dir={"": "python"},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,  # <- TODO Could this be True?
    maintainer="Felix Widmaier",
    maintainer_email="felix.widmaier@tue.mpg.de",
    description="TriFinger Robot Simulation",
    license="BSD 3-Clause",
    tests_require=["pytest"],
    # entry_points={
    #    'console_scripts': [
    #            'my_node = my_py_pkg.my_node:main'
    #    ],
    #  },
    # TODO: use entry_points instead of scripts to get rid of the .py extension
    # on the executables.
    scripts=[
        "demos/demo_cameras.py",
        "demos/demo_control.py",
        "demos/demo_inverse_kinematics.py",
        "demos/demo_load_gym_env.py",
        "demos/demo_plain_torque_control.py",
        "demos/demo_random_policy.py",
        "demos/demo_trifinger_platform.py",
        "scripts/check_position_control_accuracy.py",
        "scripts/profiling.py",
    ],
    package_data={
        "": (
            find_package_data("python/trifinger_simulation", "data")
            + find_package_data(
                "python/trifinger_simulation", "robot_properties_fingers"
            )
        )
    },
)
