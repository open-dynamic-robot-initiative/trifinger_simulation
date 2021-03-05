from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        "trifinger_simulation",
        "trifinger_simulation.gym_wrapper",
        "trifinger_simulation.gym_wrapper.envs",
    ],
    package_dir={"": "python"},
    package_data={
        "": [
            "robot_properties_fingers/meshes/*",
            "robot_properties_fingers/urdf/*",
        ]
    },
    scripts=[
        "scripts/replay_action_log.py",
        "scripts/rrc_evaluate",
        "scripts/run_evaluate_policy_all_levels.py",
        "scripts/run_replay_all_levels.py",
    ],
)

setup(**d)
