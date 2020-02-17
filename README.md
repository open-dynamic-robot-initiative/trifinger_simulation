# pyBullet Simulation for Finger Robots {#mainpage}

[TOC]

This packages provides the interface to run a pyBullet simulation with the
Finger, TriFinger or related robots.

To train an agent using implementations of RL algorithms from stable-baselines, follow the instructions below-

  0. Setup the repositories as follows- 

   * Clone treep_robotics and treep_machines_in_motion in the directory in which your workspace is

     ```
     git clone git@git-amd.tuebingen.mpg.de:robotics/treep_robotics.git
     git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
     ```
   
   * treep --clone BLMC_EI (or BLMC_EI_SIM)

1. Use the latest version of the container (named blmc_ei.def) from [here](https://git-amd.tuebingen.mpg.de/robotics/blmc_ei_singularity), and follow the steps to build the container from [Build and Run with Singularity](https://atlas.is.localnet/confluence/pages/viewpage.action?spaceKey=AMDW&title=Build+and+Run+with+Singularity), or get the latest pre-built container from [here](https://nextcloud.tuebingen.mpg.de/index.php/s/Jn5qX7NnTqJxopJ).
2. Start the singulairty shell, source the /setup.bash, perform catbuild, and source the devel/setup.bash file. This ensures that even if you make any changes to your gym environment, you do not need to install it again using pip install. And, actually, note that in case you are using multiple workspaces, do not perform a pip install. 
3. Now you need to setup stable-baselines. Clone the repo from [here](https://git-amd.tuebingen.mpg.de/sjoshi/stable-baselines/).
4. When running any script that imports the stable-baselines' modules, do it in the instance of the container in which you haven't sourced the /setup.bash. This is because otherwise it results in an import error for cv2 (an issue that's been around for years and is hopefully going to be resolved when ros is released with support for python3). 

