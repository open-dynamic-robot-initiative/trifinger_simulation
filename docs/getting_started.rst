***************
Getting Started
***************

.. contents::


Installation
============

See `installation instructions <installation.md>`_.


A Simple Demo
=============

See SimFinger :ref:`simfinger-usage-example`.


.. _sec-simulation-vs-real-robot:

Simulation vs Real Robot
========================

We aim to have the same API for controlling the robot in the ``SimFinger`` class
as in ``robot_interfaces::RobotFrontend`` to allow transition from simulation to
real robot with only minimal modifications to the code.  There are still a few
important differences one should keep in mind though.  These differences are
described in the following.

Note that it is also possible to use the simulation through the
``robot_interfaces`` structure by using a backend that uses the pyBullet driver.
This way, the full functionality of ``robot_interfaces`` is available (i.e. all
the issues described below are not there).  The drawback is that this way it is
not possible to directly access the simulation, e.g. to reset the finger.  For
more details see `robot_interfaces with Simulation`_.


Simulation is stepped in ``append_desired_action()``
----------------------------------------------------

Every time the ``append_desired_action()`` method is called, the next time step
in the simulation is computed.  This means that the state of the simulation
does not change as long as this method is not called.
This is different to the real robot, which will physically continue to move
and will repeat the last action if no new action is provided in time.


No waiting for future time steps
--------------------------------

On the real robot, it is possible to pass a time index that lies in the future
and most methods of the ``RobotFrontend`` will simply block and wait until this
time step is reached.  The simulation, however, is not running asynchronously
but is actively stepped every time ``append_desired_action()`` is called.
Therefore the behaviour of waiting for a time step is not supported.  Instead,
passing a time index that lies in the future will result in an error.


Real Time Behaviour
-------------------

As described above, the simulation is stepped in ``get_observation()``.  Each
time this function is called, the next time step is computed, based on the
``time_step`` duration passed to ``SimFinger`` during initialization.  It does
not matter how much time passes between these calls as the simulation is paused
in this time.  So your algorithm can take all the time it needs to compute the
next action.

On the real robot though, the world does not simply stop while your algorithm is
running.  The robot is controlled at a rate of 1 kHz so it expects a new action
every 1 ms.  If your algorithm is not providing the next action fast enough, the
robot will repeat the previous action until a new action is provided.


No Time Series in SimFinger
---------------------------

The ``robot_interfaces`` package makes use of time series for observations,
actions, etc.  This means all data of the last few time steps is available.  One
could, for example do the following to determine how the state of the robot
changed:

.. code-block:: python

    previous_observation = frontend.get_observation(t - 1)
    current_observation = frontend.get_observation(t)

The ``SimFinger`` class does not implement a time series, so it only provides
the observation of the current time step ``t``.  Passing any other value for
``t`` will result in an error.


API-differences to RobotFrontend
--------------------------------

Our goal is to provide the same API in ``SimFinger`` as in ``RobotFrontend`` to
make transition between simulation and real robot easy.  There are a few
differences, though.

Currently ``SimFinger`` supports the following methods:

- ``append_desired_action()``
- ``get_observation()``
- ``get_desired_action()``
- ``get_applied_action()``
- ``get_timestamp_ms()``
- ``get_current_timeindex()``

The following methods are not supported:

- ``get_status()``:  There are no meaningful values for the status message in
  simulation, so this method is omitted to avoid confusion.
- ``wait_until_timeindex()``:  In general the process of waiting for a specific
  time step is not supported, see `No waiting for future time steps`_.



robot_interfaces with Simulation
================================

It is possible to use ``robot_interfaces`` with pyBullet by using the pyBullet
driver in the backend.  By doing this, you only need to replace the backend when
switching between simulation and real robot while all the rest of your code can
remain the same.


To create a TriFinger backend using simulation:

.. code-block:: python

    import trifinger_simulation.drivers

    backend = trifinger_simulation.drivers.create_trifinger_backend(
        robot_data, real_time_mode=True, visualize=True
    )

If ``real_time_mode`` is ``True``, the backend will expect a new action every
millisecond and repeat the previous action if it is not provided in time (like
on the real robot).  If set to ``False``, it will run as fast as possible and
wait for new actions.

Set ``visualize=True`` to run the pyBullet GUI for visualization.


For a complete example, see `demo_robot_interface.py
<../demos/demo_robot_interface.py>`_.
