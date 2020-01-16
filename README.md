# pyBullet Simulation for Finger Robots {#mainpage}

[TOC]

This packages provides the interface to run a pyBullet simulation with the
Finger, TriFinger or related robots.

To run the demos in the pybullet_fingers package, please follow these steps-

  0. Setup the repositories as follows- 

   * Clone treep_robotics and treep_machines_in_motion in the directory in which your workspace is

     ```
     git clone git@git-amd.tuebingen.mpg.de:robotics/treep_robotics.git
     git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
     ```
   
   * treep --clone BLMC_EI (or BLMC_EI_SIM)

  1. Use the latest version of the container (named blmc_ei.def) from 
   [here](https://git-amd.tuebingen.mpg.de/robotics/blmc_ei_singularity).

  2. Follow the steps to build the container from [Build and Run with Singularity](https://atlas.is.localnet/confluence/pages/viewpage.action?spaceKey=AMDW&title=Build+and+Run+with+Singularity), and start the shell, do catbuild and
   source the devel/setup.bash file. This ensures that even if you make any changes to your gym environment, you do not need to install it again using pip install. And, actually, note that in case you are using multiple workspaces, do not perform a pip install. 

  3. Following demos are available as of now:
     * demo_switch.py : To demonstrate that the same commands can be used with
     both the real system and the simulated environment.
     * demo_fancy_torque_control.py : To demonstrate reaching a randomly set
     target point in the arena using torque control by directly specifying the
     position of the target only.
     * demo_move_along_a_circle.py : To move the finger uniformly along a circle.
     * demo_move_up_and_down : to show movements of all the three fingers going up and down
     
  4. The gym-wrapper is a work in progress and exists now for structural testing. A few slight modifications have to be added as compared to the environment in  python/pybullet_fingers/sim_finger. This will be updated shortly for a reaching task. 

  5. Now the gym interface doesn't have to be installed explicitly by running pip install. It is installed via catmake as a sub-package of pybullet_fingers. It gets installed by default. To test it run python3 inside the container, then:

   ```python
   import gym
   import pybullet_fingers
   env = gym.make('pybullet_fingers.gym_wrapper:finger-v0')
   ```

  6. Now the torque safety checks have been implemented as they are on the real finger. So, the torques are constrained to stay within the maximum motor torque limits of [-0.36, 0.36] Nm. The torque profile (here, first shown for 3, then for 50 different positions to reach, each of 2000 iterations = 2s, running at 1 KHz)  looks like this-
   ![3: Torque vs iterations](https://git-amd.tuebingen.mpg.de/robotics/pybullet_fingers/blob/sjoshi/sim_wip/docs/torque_check.png), ![100: Torque v/s iterations](https://git-amd.tuebingen.mpg.de/robotics/pybullet_fingers/blob/sjoshi/sim_wip/docs/torque100.png)

   ​	So, basically maximum torque needs to be applied almost all the time. The gains are adjusted to be:  kp = 20, kd = 7, and safety_kd = 1.4
