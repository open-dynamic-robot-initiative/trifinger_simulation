Installation
================

There are two ways to build/install this package.

1. As an isolated Python package
2. As part of a catkin workspace

Install as Python package in a conda environment
----------------------------------------------------

Prerequisites: Install Anaconda or Miniconda
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If not already done, install ``conda`` (Miniconda is sufficient).  To do so, see the
`official documentation <https://docs.conda.io/projects/conda/en/latest/user-guide/install/>`_.

We have tested with conda version 4.8.3.

Install the trifinger_simulation package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Clone this repo and cd into it::

       git clone https://gitlab.is.tue.mpg.de/robotics/trifinger_simulation.git
       cd trifinger_simulation

2. Set up the conda env::

       export PYTHONNOUSERSITE=True
       conda env create -f environment.yml

   Note that the environment.yml contains some packages (such as
   stable-baselines and tensorflow) which are only required for running the
   examples we provide. If you do not wish to install them, you can safely remove
   them, see comments in the environment.yml file.

3. Activate the conda env (you might have to start a new terminal)::

       conda activate trifinger_simulation

4. Install the trifinger_simulation package::

       python -m pip install -e .

     **Note on Activating the Conda Environment**

    Every time you activate your trifinger_simulation conda environment, it is
    important that you make sure that ``PYTHONNOUSERSITE`` is set to True, as is
    done in step 2 above.  This ensures that your conda environment is isolated
    from the global/user site-packages, which is important for us to reproduce
    your env on our side. For convenience, we provide a script which does this::

        source conda_activate_trifinger_simulation.sh


Build using catkin
-------------------------

``trifinger_simulation`` can also be used as a catkin package together with
``robot_interfaces``, etc.

Proper instructions for this will follow soon.