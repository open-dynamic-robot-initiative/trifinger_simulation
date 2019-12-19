# pyBullet Simulation for Finger Robots {#mainpage}

[TOC]

This packages provides the interface to run a pyBullet simulation with the
Finger, TriFinger or related robots.

To run the demos in the pybullet_fingers package, please follow these steps-

0. Setup the repositories as follows- (as described by Felix)

   * Delete treep_amd_clmc from your workspace 

   * Add treep_robotics at the top level of our workspace: 

     ```
     git clone [git@git-amd.tuebingen.mpg.de:robotics/treep_robotics](mailto:git@git-amd.tuebingen.mpg.de:robotics/treep_robotics) 
     ```

   * Update the origins of your local repositories: 

     ```
     treep --fix-origins 
     ```

   *     cd .../treep_robotics; git pull 
         treep --clone BLMC_EI 


1. Use the latest version of the container (named blmc_ei.def) from the wiki
   page (find below for reference)

  ~~~~markdown
  Bootstrap: docker
  From: osrf/ros:kinetic-desktop

  # To save time when rebuilding the image, you can pull the ROS image once and
  # then use the following instead:
  #Bootstrap: localimage
  #From: ./ros_kinetic-desktop.sif

  %post
      apt-get update
      apt-get dist-upgrade -y

      apt-get install -y clang clang-format

      # missing dependencies
      apt-get install -y \
          wget \
          freeglut3-dev \
          ros-kinetic-realtime-tools \
          python3-pip \
          python3-progressbar \
          python3-empy \
          python3-numpy \
          python3-yaml \
          libcereal-dev \
          libopencv-dev  # FIXME fix code so that ROS-version of OpenCV is used

      # Need Python 3 version of several packages to build with Python 3
      pip3 --no-cache-dir install catkin-pkg rospkg catkin_tools

      # for building documentation
      pip3 --no-cache-dir install doxypypy

      # clean up
      apt-get clean

      # create a setup file
      echo ". /opt/ros/kinetic/setup.bash
  alias catmake='catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3'
  alias catbuild='catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3'
  " > /setup.bash


  %labels
      Version 0.4.0

  %help
      Container for building the BLMC_EI project.
      Run it with `singularity shell` at the root of your workspace, set up the
      environment by executing `source /setup.sh` and build with `catbuild` (which
      is an alias for `catkin build` that sets the Python executable to use Python
      3) or `catmake` (same but using `catkin_make`).
  ~~~~

  2. Bootstrap the above container with the one below (call this say,
     blmc_ei_sim.def).

~~~~markdown
Bootstrap: localimage
From: ./blmc_ei.sif

%post
    #Install PyBullet
    pip3 install pybullet

    #Install Gym
    pip3 install gym
~~~~

  3. Follow the steps to build the container and start the shell, do catmake and
     source the devel/setup.bash file, and then run 'rosrun pybullet_fingers <demo_file>'.

  4. Following demos are available as of now:
     * demo_switch.py : To demonstrate that the same commands can be used with
     both the real system and the simulated environment.
     * demo_fancy_torque_control.py : To demonstrate reaching a randomly set
     target point in the arena using torque control by directly specifying the
     position of the target only.
     * demo_move_along_a_circle.py : To move the finger uniformly along a circle.

5. The gym-wrapper is a work in progress and exists now for structural testing. A few slight modifications have to be added as compared to the environment in  python/pybullet_fingers/sim_finger. 

6. Now the gym interface doesn't have to be installed explicitly by running pip install. It is installed via catmake as a sub-package of pybullet_fingers. It gets installed by default. To test it run python3 inside the container, then:

   ```python
   import gym
   import pybullet_fingers
   env = gym.make('pybullet_fingers.gym_wrapper:finger-v0')
   ```

7.  Now the torque safety checks have been implemented as they are on the real finger. So, the torques are constrained to stay within the maximum motor torque limits of [-0.36, 0.36] Nm. The torque profile (here, first shown for 3, then for 50 different positions to reach, each of 2000 iterations = 2s, running at 1 KHz)  looks like this-
   ![3: Torque vs iterations](https://git-amd.tuebingen.mpg.de/robotics/pybullet_fingers/blob/sjoshi/sim_wip/docs/torque_check.png), ![100: Torque v/s iterations](https://git-amd.tuebingen.mpg.de/robotics/pybullet_fingers/blob/sjoshi/sim_wip/docs/torque100.png)

   ​	So, basically maximum torque needs to be applied almost all the time. The gains are adjusted to be:  kp = 20, kd = 7, and safety_kd = 1.4