from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        "pybullet_fingers",
        "pybullet_fingers.gym_wrapper",
        "pybullet_fingers.gym_wrapper.envs",
    ],
    package_dir={"": "python"},
    package_data={
        "": [
            "robot_properties_fingers/meshes/stl/*",
            "robot_properties_fingers/urdf/*",
        ]
    },
)

setup(**d)
