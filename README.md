# TriFinger robot simulation

Welcome to the official simulation of the TriFinger robots!

To know more about the TriFinger robots, check out our official [project website](https://sites.google.com/view/trifinger), and the [preprint](https://arxiv.org/abs/2008.03596) of this work.

## Installation instructions

### Prerequisites: Install Anaconda or Miniconda

If not already done, install ``conda`` (Miniconda is sufficient).  To do so, see the
`official documentation <https://docs.conda.io/projects/conda/en/latest/user-guide/install/>`_.

We have tested with conda version 4.8.3.

## Install the trifinger_simulation_package

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
