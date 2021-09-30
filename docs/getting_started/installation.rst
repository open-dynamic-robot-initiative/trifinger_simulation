Installation
============

There are two ways to build/install this package:

1. Stand-alone installation using pip
2. As part of a colon workspace


Stand-alone installation using pip
----------------------------------

.. note::

   We are developing for Python 3.8.  Other versions may work as well but are
   not officially supported.


Get the source from GitHub::

    $ git clone https://github.com/open-dynamic-robot-initiative/trifinger_simulation


To avoid version conflicts with other packages on your system, it is
recommended to install the package in an isolated environment like venv or
conda.


Using venv
~~~~~~~~~~

You may first need to install ``venv``.  E.g. on Ubuntu: ``sudo apt install
python3-venv``.  Then create a new environment and install the package and its
dependencies

.. code-block:: bash

    $ python3 -m venv ~/venv_trifinger_simulation
    $ . ~/venv_trifinger_simulation/bin/activate

    $ pip install --upgrade pip  # make sure the latest version of pip is used

    $ cd trifinger_simulation
    $ pip install -r requirements.txt
    $ pip install .


Using conda
~~~~~~~~~~~

If not already done, install ``conda`` (Miniconda is sufficient).  To do so, see the
`official documentation <https://docs.conda.io/projects/conda/en/latest/user-guide/install/>`_.

We tested with conda version 4.8.3.

1. Create the conda environment::

       $ conda env create -f environment.yml

2. Activate the environment (you may have to do this in a new terminal)::

       $ conda activate trifinger_simulation

3. Install the trifinger_simulation package::

       $ cd trifinger_simulation
       $ python3 -m pip install .


Test Installation
~~~~~~~~~~~~~~~~~

You can test the installation by running the unit tests::

    $ python3 -m pytest tests/

or by running one of the demos::

    $ python3 demos/demo_trifinger_platform.py



.. _`colcon`:

Using colcon
------------

trifinger_simulation is part of the "ROBOT_FINGERS" project.  For build
instructions see the :doc:`robot_fingers documentation
<robot_fingers:doc/installation>`.
