from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['pybullet_fingers', 'pybullet_fingers.gym_wrapper'],
        package_dir={'': 'python'},
)

setup(**d)
