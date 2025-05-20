Installation
============

There are two ways to build/install this package:

1. Stand-alone installation using pip
2. As part of a colon workspace


Stand-alone installation using pip
----------------------------------

.. note::

   We are developing for Python 3.10.  Other versions may work as well but are
   not officially tested.


To install the latest release:

.. code-block:: bash

   $ pip install trifinger-simulation

If you run into errors during installation, you may need to update pip first:

.. code-block:: bash

   $ pip install --upgrade pip


.. note::

    To avoid version conflicts with other packages on your system, it is
    recommended to install the package in an isolated environment, e.g. using venv:

    .. code-block:: bash

        $ python3 -m venv ~/venv_trifinger_simulation
        $ . ~/venv_trifinger_simulation/bin/activate

        $ pip install --upgrade pip  # make sure the latest version of pip is used
        $ pip install trifinger-simulation


Installation from source
~~~~~~~~~~~~~~~~~~~~~~~~

To install the latest version from source, follow the same instructions as above
but replace ``pip install trifinger-simulation`` with

.. code-block:: bash

    $ git clone https://github.com/open-dynamic-robot-initiative/trifinger_simulation
    $ cd trifinger_simulation
    $ pip install .


Test Installation
~~~~~~~~~~~~~~~~~

You can test the installation by running the unit tests::

    $ python3 -m pytest tests/

or by running one of the demos::

    $ python3 demos/demo_trifinger_platform.py



.. _`colcon`:

Using colcon
------------

The ``trifinger_simulation`` package is also part of the TriFinger software
bundle.  See :ref:`trifinger_docs:install_software` in the TriFinger
documentation.
