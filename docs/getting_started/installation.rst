Installation
============

There are two ways to build/install this package.

1. Using conda/pip
2. Using colcon


Using conda
-----------

Prerequisites: Install Anaconda or Miniconda
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If not already done, install ``conda`` (Miniconda is sufficient).  To do so, see the
`official documentation <https://docs.conda.io/projects/conda/en/latest/user-guide/install/>`_.

We tested with conda version 4.8.3.


Installing the trifinger_simulation package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Clone this repo and cd into it::

       git clone https://github.com/open-dynamic-robot-initiative/trifinger_simulation
       cd trifinger_simulation

2. Set up the conda env::

       conda env create -f environment.yml

   Note that the environment.yml contains some packages (such as
   stable-baselines and tensorflow) which are only required for running the
   examples we provide. If you do not wish to install them, you can safely remove
   them, see comments in the environment.yml file.

3. Activate the conda env (you might have to start a new terminal)::

       conda activate trifinger_simulation

4. Install the trifinger_simulation package::

       python -m pip install -e .


.. _`colcon`:

Using colcon
------------

trifinger_simulation is part of the "ROBOT_FINGERS" project.  For build
instructions see the `robot_fingers documentation
<https://open-dynamic-robot-initiative.github.io/code_documentation/robot_fingers/docs/doxygen/html/md_doc_installation.html>`_
