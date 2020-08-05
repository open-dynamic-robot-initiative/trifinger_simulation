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
switch to catkin, you need to remove the package first, otherwise it can
interfere with the environment of the catkin workspace.

1. Clone this repo,

       git clone git@gitlab.is.tue.mpg.de:robotics/trifinger_simulation.git

2. Then run the installation script in interactive mode. The installation
   script sets up a conda env (based on Python 3.6.9) and installs the
   `trifinger_simulation` package within it.

       ${SHELL} -i create_conda_env.sh

3. Then activate the env,

       conda activate trifinger_simulation

4. You should check that you the tests in `tests/` (which wouldn't include the
   tests in `tests/catkin`) are successful:

       python -m unittest discover tests/

   Note that `unittest discover` doesn't search recursively and hence the tests
   in `tests/catkin` won't get executed.
