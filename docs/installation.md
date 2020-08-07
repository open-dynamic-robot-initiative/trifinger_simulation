Installation
============

There are two ways to build/install this package.

1. As part of a catkin workspace (recommended)
2. As an isolated Python package using the setup.py

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

1. Clone this repo,

       git clone git@gitlab.is.tue.mpg.de:robotics/trifinger_simulation.git

2. Then run the installation script in interactive mode. The installation
   script sets up a conda env (based on Python 3.6.9) and installs the
   `trifinger_simulation` package within it.

       ${SHELL} -i create_conda_env.sh

3. Then activate the env using,

       source conda_activate_trifinger_simulation.sh

**VERY VERY IMPORTANT** Everytime you want to activate the `trifinger_simulation` conda environment, you must do so using the above command. It ensures that your conda environment is isolated from the global/user site-packages, and enables one to know exactly what they are running inside their environment. In case you do not activate the environment like this, and instead just activate it using `conda activate trifinger_simulation`, you would still be able to access the packages in your global/user site-packages from inside an activated conda environment.

*Why is this important?* By having access only to the packages you install from within the environment `trifinger_simulation` using conda/pip, when you export this (active) environment as a yml, say using,

       conda env export > environment.yml

you will get a list of all the packages installed within the environment along with their versions, which you used to run your code. This ensures reproducibility of your code by using this yml.