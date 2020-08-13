.. _sec-simulation-vs-real-robot:

**************************
Simulation vs Real Robot
**************************
The ``SimFinger`` class provides us with an API to control the TriFinger robots in
simulation. And, how can these robots be controlled in real life? via the
`robot_interfaces::RobotFrontend`_ class.

We aim to have the same API for controlling the robot in the ``SimFinger`` class
as in ``robot_interfaces::RobotFrontend`` to allow transition from simulation to
real robot with only minimal modifications to the code. There are still a few
important differences one should keep in mind though. These differences are
described below:

.. note:: 

  It is also possible to use our simulation through ``robot_interfaces``
  by using a backend that uses the pyBullet driver. This way, the full
  functionality of ``robot_interfaces`` in addition to that provided by
  ``SimFinger`` is available (so you won't have to keep the below
  differences in mind betwen the simulation and the real robot). 
  However, a drawback of using this approach is that this way it is
  not possible to directly access the simulation, e.g. to reset the finger.
  For more details see :ref:`robot_interfaces with Simulation`.



Simulation is stepped in ``append_desired_action()``
========================================================

The simulation is explicitly stepped in the ``append_desired_action()``
method: this is because the simulation doesn't exhibit real-time
behaviour. So, every time the ``append_desired_action()`` is called,
the simulation is stepped, and the next time step in the simulation is computed.
This means that the state of the simulation does not change as long as this
method is not called. This is different than on the real robot, which will physically
continue to move and will repeat the last action if no new action is provided in time.


No waiting for future time steps
======================================

On the real robot, it is possible to pass a time index that lies in the future
and most methods of the ``RobotFrontend`` will simply block and wait until this
time step is reached.  The simulation, however, is not running asynchronously
but is actively stepped every time ``append_desired_action()`` is called.
Therefore the behaviour of waiting for a time step is not supported.  Instead,
passing a time index that lies in the future will result in an error.


No Time Series in SimFinger
==============================

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


API-differences with RobotFrontend
====================================

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

.. _`robot_interfaces::RobotFrontend`: https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/include/robot_interfaces/robot_frontend.hpp
