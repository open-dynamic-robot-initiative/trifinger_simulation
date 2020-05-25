Installation
============

There are two ways to build/install this package.

1. As part of a catkin workspace (recommended)
2. As an isolated Python package using the setup.py

In general, Option 1 should be preferred, ideally using Singularity to avoid any
dependency issues.  Only fall back to Option 2 if 1 does not work for some
reason.


Build using catkin
------------------

Setting up a workspace and building it via our Singularity image is described
[here in the internal
wiki](https://atlas.is.localnet/confluence/display/AMDW/Getting+Started+with+BLMC+EI+Software).
If you only want to use the simulation (i.e. not the real robot), you can
replace `BLMC_EI` with `BLMC_EI_SIM` in the `treep --clone` command:

    treep --clone BLMC_EI_SIM

After building the workspace, sourcing it's `setup.bash` is enough to setup


Install as Python package in a conda environment
------------------------------------------------

**Note:** Only do this if you cannot use the catkin approach (see above).  If
you installed the package using the following instructions and later want to
switch to catkin, you need to remove the package first, otherwise it can infer
with the environment of the catkin workspace.


1. Clone this repo and then create it's conda environment to install all
   dependencies:

       git clone git@gitlab.is.tue.mpg.de:robotics/pybullet_fingers.git
       cd pybullet_fingers
       conda env create -f environment.yml

2. Install the pybullet_fingers package inside the (pybullet_fingers) conda env:

       conda activate pybullet_fingers
       (pybullet_fingers) python -m pip install .
