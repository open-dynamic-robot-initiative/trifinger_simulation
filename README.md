# pyBullet Simulation for Finger Robots {#mainpage}

[TOC]

This package provides the interface to run a pyBullet simulation with the
Finger, TriFinger or related robots. It can be either [pip-installed](#install-as-a-pip-package-in-a-conda-env),
or used in a catkin workspace (preferably using singularity to avoid manual installation
of every single dependency, as is illustrated [here](#the-singularity-approach). Note that the latter method
of installation is recommended to ensure the latest robot models are used.

## Install as a pip package in a conda env

1. Clone this repo and then create it's conda environment to install all dependencies.

  ```bash
  git clone git@git-amd.tuebingen.mpg.de:robotics/pybullet_fingers.git
  cd pybullet_fingers
  conda env create -f pybullet_fingers_linux_cpu.yml
  ```

2. Install the pybullet_fingers package inside the (pybullet_fingers) conda env.

  ```bash
  conda activate pybullet_fingers
  (pybullet_fingers) python -m pip install .
  ```

## The Singularity Approach

To train an agent using implementations of RL algorithms from stable-baselines, follow the instructions below-

  1. Setup the repositories in your workspace as follows-

      ```bash
      git clone git@git-amd.tuebingen.mpg.de:robotics/treep_robotics.git
      git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
      treep --clone BLMC_EI_SIM
      ```

      You may also need stable_baselines. Clone the repo from [here](https://git-amd.tuebingen.mpg.de/sjoshi/stable-baselines/).

  2. Use the latest version of the container (named blmc_ei.def) from [here](https://git-amd.tuebingen.mpg.de/robotics/blmc_ei_singularity), and follow the steps to build the container from [Build and Run with Singularity](https://atlas.is.localnet/confluence/pages/viewpage.action?spaceKey=AMDW&title=Build+and+Run+with+Singularity), or get the latest pre-built container from [here](https://nextcloud.tuebingen.mpg.de/index.php/s/Jn5qX7NnTqJxopJ).

  3. Then, after follow these steps to start the container-

      ```bash
      singularity shell path/to/container.sif
      source /setup.bash
      catbuild
      source path/to/workspace/devel/setup.bash
      ```
  
      This ensures that even if you make any changes to your gym environment, you do not need to install it again using pip install. And, actually, note that in case you are using multiple workspaces, do not perform a pip install.

  4. When running any script that imports the stable-baselines' modules, do it in the instance of the container in which you haven't sourced the /setup.bash. This is because otherwise it results in an import error for cv2 (an issue that's been around for years and is hopefully going to be resolved when ros is released with support for python3).
